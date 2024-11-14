#include "radar_odometry/radar_odometry.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace RadarOdometry {
gtsam::Vector BodyFrameVelocityFactor::evaluateError(
    const gtsam::Pose3 &pose, const gtsam::Vector3 &vel,
    boost::optional<gtsam::Matrix &> H1,
    boost::optional<gtsam::Matrix &> H2) const {
  gtsam::Vector3 predictedVel = pose.rotation().transpose() * vel;
  if (H1) {
    *H1 = gtsam::Matrix::Zero(3, 6);
    H1->block<3, 3>(0, 0) = gtsam::skewSymmetric(predictedVel);
  }
  if (H2) {
    *H2 = pose.rotation().transpose();
  }

  return predictedVel - measuredVel_;
}

IMUPreintegration::IMUPreintegration() {
  mSubImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000,
                                           &IMUPreintegration::imuHandler, this,
                                           ros::TransportHints().tcpNoDelay());
  mSubFrontOdometry = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(
      velocityOdomTopic, 20,
      boost::bind(&IMUPreintegration::odometryHandler, this, _1,
                  velocityOdomTopic),
      ros::VoidPtr(), ros::TransportHints().tcpNoDelay());

  if (useRearRadar) {
    mSubRearOdometry = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(
        rearVelocityOdomTopic, 20,
        boost::bind(&IMUPreintegration::odometryHandler, this, _1,
                    rearVelocityOdomTopic),
        ros::VoidPtr(), ros::TransportHints().tcpNoDelay());
  }

  mPubImuOdometry =
      nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);

  boost::shared_ptr<gtsam::PreintegrationParams> p =
      gtsam::PreintegrationParams::MakeSharedU(imuGravity);
  p->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      pow(imuAccNoise, 2); // acc white noise in continuous
  p->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      pow(imuGyrNoise, 2); // gyro white noise in continuous
  p->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      pow(1e-4, 2); // error committed in integrating position from velocities
  gtsam::imuBias::ConstantBias prior_imu_bias(
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  // assume zero initial bias

  mPriorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2)
          .finished()); // rad,rad,rad,m, m, m
  mPriorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
  mPriorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  mRobustNoiseModel = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.0),
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(3) << 0.2, 0.2, 0.2).finished())); // m/s, m/s, m/s
  mNoiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN,
                            imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN)
                               .finished();

  mImuIntegratorOpt_ =
      new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
  mPrevPose_ = gtsam::Pose3();
  mPrevVel_ = gtsam::Vector3(0, 0, 0);
  mPrevBias_ = gtsam::imuBias::ConstantBias();
}

void IMUPreintegration::resetOptimization() {
  gtsam::ISAM2Params optParameters;
  optParameters.relinearizeThreshold = 0.1;
  optParameters.relinearizeSkip = 1;
  if (useQRFactorization)
    optParameters.factorization = gtsam::ISAM2Params::QR;
  mOptimizer = gtsam::IncrementalFixedLagSmoother(smootherLag, optParameters);

  gtsam::NonlinearFactorGraph newGraphFactors;
  mGraphFactors = newGraphFactors;

  gtsam::Values NewGraphValues;
  mGraphValues = NewGraphValues;

  gtsam::FixedLagSmoother::KeyTimestampMap newKeyTimestampMap;
  mKeyTimestampMap = newKeyTimestampMap;
}

void IMUPreintegration::resetParams() {
  mLastImuMsgTime = -1;
  mDoneFirstOpt = false;
  mSystemInitialized = false;
}

void IMUPreintegration::odometryHandler(
    const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &odomMsg,
    const std::string &topicName) {
  std::lock_guard<std::mutex> lock(mMutex);

  geometry_msgs::TwistWithCovarianceStamped thisOdom = (*odomMsg);
  mOdomMap[thisOdom.header.stamp] = thisOdom;
}

void IMUPreintegration::optimize(
    const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &odomMsg) {
  double currentCorrectionTime = ROS_TIME(odomMsg);

  // make sure we have imu data to integrate
  if (mImuQueOpt.empty()) {
    return;
  }

  float vX = odomMsg->twist.twist.linear.x;
  float vY = odomMsg->twist.twist.linear.y;
  float vZ = odomMsg->twist.twist.linear.z;
  gtsam::Vector3 vel(vX, vY, vZ);

  // 0. initialize system
  if (mSystemInitialized == false) {
    resetOptimization();

    // pop old IMU message
    while (!mImuQueOpt.empty()) {
      if (ROS_TIME(&mImuQueOpt.front()) < currentCorrectionTime) {
        mLastImuOptTime = ROS_TIME(&mImuQueOpt.front());
        mImuQueOpt.pop_front();
      } else
        break;
    }
    if (mLastImuOptTime == -1) {
      return;
    }

    // initial pose
    /* mPrevPose_ = gtsam::Pose3(); */
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), mPrevPose_,
                                               mPriorPoseNoise);
    mGraphFactors.add(priorPose);
    // initial velocity
    /* mPrevVel_ = gtsam::Vector3(0, 0, 0); */
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), mPrevVel_,
                                                mPriorVelNoise);
    mGraphFactors.add(priorVel);
    // initial bias
    /* mPrevBias_ = gtsam::imuBias::ConstantBias(); */
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), mPrevBias_,
                                                               mPriorBiasNoise);
    mGraphFactors.add(priorBias);
    // add values
    mGraphValues.insert(X(0), mPrevPose_);
    mGraphValues.insert(V(0), mPrevVel_);
    mGraphValues.insert(B(0), mPrevBias_);
    // add keytimestamp
    mKeyTimestampMap[X(0)] = currentCorrectionTime;
    mKeyTimestampMap[V(0)] = currentCorrectionTime;
    mKeyTimestampMap[B(0)] = currentCorrectionTime;
    // optimize once
    mOptimizer.update(mGraphFactors, mGraphValues, mKeyTimestampMap);
    mGraphFactors.resize(0);
    mGraphValues.clear();
    mKeyTimestampMap.clear();

    mImuIntegratorOpt_->resetIntegrationAndSetBias(mPrevBias_);

    mKey = 1;
    mSystemInitialized = true;

    return;
  }

  sensor_msgs::Imu lerpedImu;
  // 1. integrate imu data and optimize
  while (!mImuQueOpt.empty()) {
    // pop and integrate imu data that is between two optimizations
    sensor_msgs::Imu *thisImu = &mImuQueOpt.front();
    sensor_msgs::Imu *lastImu = &mImuQueOpt.front();
    double imuTime = ROS_TIME(thisImu);
    double dt =
        (mLastImuOptTime < 0) ? (1.0 / 100.0) : (imuTime - mLastImuOptTime);
    if (imuTime < currentCorrectionTime) {
      mImuIntegratorOpt_->integrateMeasurement(
          gtsam::Vector3(thisImu->linear_acceleration.x,
                         thisImu->linear_acceleration.y,
                         thisImu->linear_acceleration.z),
          gtsam::Vector3(thisImu->angular_velocity.x,
                         thisImu->angular_velocity.y,
                         thisImu->angular_velocity.z),
          dt);

      mLastImuOptTime = imuTime;
      lastImu = thisImu;
      mImuQueOpt.pop_front();
    } else {
      double s =
          (imuTime - currentCorrectionTime) / (imuTime - mLastImuOptTime);
      lerpedImu = Lerp(*lastImu, *thisImu, s);
      mImuIntegratorOpt_->integrateMeasurement(
          gtsam::Vector3(lerpedImu.linear_acceleration.x,
                         lerpedImu.linear_acceleration.y,
                         lerpedImu.linear_acceleration.z),
          gtsam::Vector3(lerpedImu.angular_velocity.x,
                         lerpedImu.angular_velocity.y,
                         lerpedImu.angular_velocity.z),
          dt);
      break;
    }
  }

  // add imu factor to graph
  const gtsam::PreintegratedImuMeasurements &preint_imu =
      dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
          *mImuIntegratorOpt_);
  gtsam::ImuFactor imu_factor(X(mKey - 1), V(mKey - 1), X(mKey), V(mKey),
                              B(mKey - 1), preint_imu);
  mGraphFactors.add(imu_factor);
  // add imu bias between factor
  mGraphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(mKey - 1), B(mKey), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(sqrt(mImuIntegratorOpt_->deltaTij()) *
                                          mNoiseModelBetweenBias)));

  gtsam::Vector3 measuredVel = gtsam::Vector3(vX, vY, vZ);
  gtsam::NonlinearFactor::shared_ptr velFactor(new BodyFrameVelocityFactor(
      X(mKey), V(mKey), measuredVel, mRobustNoiseModel));
  mGraphFactors.add(velFactor);
  // insert predicted values
  gtsam::NavState propState_ =
      mImuIntegratorOpt_->predict(mPrevState_, mPrevBias_);
  mGraphValues.insert(X(mKey), propState_.pose());
  mGraphValues.insert(V(mKey), propState_.v());
  mGraphValues.insert(B(mKey), mPrevBias_);
  // add keytimestamp
  mKeyTimestampMap[X(mKey)] = currentCorrectionTime;
  mKeyTimestampMap[V(mKey)] = currentCorrectionTime;
  mKeyTimestampMap[B(mKey)] = currentCorrectionTime;
  // optimize
  mOptimizer.update(mGraphFactors, mGraphValues, mKeyTimestampMap);
  mOptimizer.update();
  mGraphFactors.resize(0);
  mGraphValues.clear();
  mKeyTimestampMap.clear();
  // Overwrite the beginning of the preintegration for the next step.
  gtsam::Values result = mOptimizer.calculateEstimate();
  mPrevPose_ = result.at<gtsam::Pose3>(X(mKey));
  mPrevVel_ = result.at<gtsam::Vector3>(V(mKey));
  mPrevState_ = gtsam::NavState(mPrevPose_, mPrevVel_);
  mPrevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(mKey));
  // Reset the optimization preintegration object.
  mImuIntegratorOpt_->resetIntegrationAndSetBias(mPrevBias_);
  // check optimization
  if (failureDetection(mPrevVel_, mPrevBias_)) {
    resetParams();
    return;
  }

  mKey++;
  mDoneFirstOpt = true;
  // publish odometry
  nav_msgs::Odometry odometry;
  odometry.header.stamp = odomMsg->header.stamp;
  odometry.header.frame_id = odometryFrame;
  odometry.child_frame_id = "imu";
  odometry.pose.pose.position.x = mPrevPose_.translation().x();
  odometry.pose.pose.position.y = mPrevPose_.translation().y();
  odometry.pose.pose.position.z = mPrevPose_.translation().z();
  odometry.pose.pose.orientation.x = mPrevPose_.rotation().toQuaternion().x();
  odometry.pose.pose.orientation.y = mPrevPose_.rotation().toQuaternion().y();
  odometry.pose.pose.orientation.z = mPrevPose_.rotation().toQuaternion().z();
  odometry.pose.pose.orientation.w = mPrevPose_.rotation().toQuaternion().w();

  odometry.twist.twist.linear.x = mPrevState_.velocity().x();
  odometry.twist.twist.linear.y = mPrevState_.velocity().y();
  odometry.twist.twist.linear.z = mPrevState_.velocity().z();
  odometry.twist.twist.angular.x =
      lerpedImu.angular_velocity.x + mPrevBias_.gyroscope().x();
  odometry.twist.twist.angular.y =
      lerpedImu.angular_velocity.y + mPrevBias_.gyroscope().y();
  odometry.twist.twist.angular.z =
      lerpedImu.angular_velocity.z + mPrevBias_.gyroscope().z();
  mPubImuOdometry.publish(odometry);
  if (mKey > 100) {
    resetParams();
    resetOptimization();
  }
}

bool IMUPreintegration::failureDetection(
    const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
  Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
  if (vel.norm() > 40) {
    ROS_WARN("Large velocity, reset IMU-preintegration!");
    return true;
  }

  Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                     biasCur.accelerometer().z());
  Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                     biasCur.gyroscope().z());
  if (ba.norm() > 1.0 || bg.norm() > 1.0) {
    ROS_WARN("Large bias, reset IMU-preintegration!");
    return true;
  }

  return false;
}

void IMUPreintegration::imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  std::lock_guard<std::mutex> lock(mMutex);

  sensor_msgs::Imu thisImu = (*imuMsg);

  mImuQueOpt.push_back(thisImu);

  double bufferTime = 0.1;
  auto odomFront = mOdomMap.begin();
  if (odomFront == mOdomMap.end()) {
    return;
  } else if (ROS_TIME(imuMsg) - odomFront->first.toSec() < bufferTime) {
    return;
  } else {
    geometry_msgs::TwistWithCovarianceStamped::ConstPtr odomMsg =
        boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>(
            odomFront->second);
    optimize(odomMsg);
    mOdomMap.erase(odomFront);
  }
}
} // namespace RadarOdometry
