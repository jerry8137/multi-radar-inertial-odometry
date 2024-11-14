#include "radar_odometry/utility.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <tf_conversions/tf_eigen.h>

namespace RadarOdometry {
class BodyFrameVelocityFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
  gtsam::Vector3 measuredVel_;

public:
  BodyFrameVelocityFactor(const gtsam::Key &poseKey, const gtsam::Key &velKey,
                          const gtsam::Vector3 &measuredVel,
                          const gtsam::SharedNoiseModel &model)
      : NoiseModelFactor2(model, poseKey, velKey), measuredVel_(measuredVel) {}
  gtsam::Vector
  evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const;
};

class IMUPreintegration : public ParamServer {
private:
  std::mutex mMutex;

  ros::Subscriber mSubImu;
  ros::Subscriber mSubFrontOdometry;
  ros::Subscriber mSubRearOdometry;
  ros::Publisher mPubImuOdometry;

  bool mSystemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr mPriorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr mPriorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr mPriorBiasNoise;
  gtsam::noiseModel::Robust::shared_ptr mRobustNoiseModel;
  gtsam::Vector mNoiseModelBetweenBias;

  gtsam::PreintegratedImuMeasurements *mImuIntegratorOpt_;

  std::deque<sensor_msgs::Imu> mImuQueOpt;
  std::deque<sensor_msgs::Imu> mImuQueImu;

  gtsam::Pose3 mPrevPose_;
  gtsam::Vector3 mPrevVel_;
  gtsam::NavState mPrevState_;
  gtsam::imuBias::ConstantBias mPrevBias_;

  gtsam::NavState mPrevStateOdom;
  gtsam::imuBias::ConstantBias mPrevBiasOdom;

  bool mDoneFirstOpt = false;
  double mLastImuMsgTime = -1;
  double mLastImuOptTime = -1;

  /* gtsam::ISAM2 optimizer; */
  gtsam::IncrementalFixedLagSmoother mOptimizer;
  gtsam::NonlinearFactorGraph mGraphFactors;
  gtsam::Values mGraphValues;
  gtsam::FixedLagSmoother::KeyTimestampMap mKeyTimestampMap;

  int mKey = 1;

  std::deque<geometry_msgs::TwistWithCovarianceStamped> mFrontOdomQue;
  std::deque<geometry_msgs::TwistWithCovarianceStamped> mRearOdomQue;
  std::map<ros::Time, geometry_msgs::TwistWithCovarianceStamped> mOdomMap;

public:
  explicit IMUPreintegration();
  void resetOptimization();
  void resetParams();
  void odometryHandler(
      const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &odomMsg,
      const std::string &topicName);
  bool failureDetection(const gtsam::Vector3 &velCur,
                        const gtsam::imuBias::ConstantBias &biasCur);
  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuRaw);
  void optimize(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &odomMsg);
};

class TransformFusion : public ParamServer {
private:
  std::mutex mMutex;

  ros::Subscriber mSubImuOdometry;

  ros::Publisher mPubImuOdometry;
  ros::Publisher mPubImuPath;

  tf::TransformListener mTfListener;
  tf::StampedTransform mMapToOdom;

public:
  explicit TransformFusion();
  void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg);
};
} // namespace RadarOdometry
