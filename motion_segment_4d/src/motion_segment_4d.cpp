#include <motion_segment_4d/motion_segment_4d.h>

namespace MS4D {

MotionSegment4D::MotionSegment4D(ros::NodeHandle &nh) {
  mNh = nh;

  GetParameter();

  mPublishFrame =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(mInputTopic, mNh)
          ->header.frame_id;

  mRadarCloudPub = mNh.advertise<PointCloud>("radar_stationary_cloud", 100);
  mRadarNonStationaryCloudPub =
      mNh.advertise<PointCloud>("radar_non_stationary_cloud", 100);
  mTargetVelocityPub =
      mNh.advertise<visualization_msgs::MarkerArray>("target_velocity", 100);
  mEgoVelocityPub = mNh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
      "ego_velocity", 100);

  mContiRadarSub =
      mNh.subscribe(mInputTopic, 100, &MotionSegment4D::ContiCallback, this);
  mVehicleStateSub = mNh.subscribe(
      "/vehicle_state", 100, &MotionSegment4D::VehicleStateCallback, this);

  mGetVehicleState = false;
  mUseRadarTime = std::nullopt;

  mRadarTransformStamped = GetTransformStamped("base_link", mPublishFrame);
  mRadarToImuTransformStamped = GetTransformStamped(mPublishFrame, "imu");

  // v_radar = Ad_Trb * v_baselink
  mAdjointMat = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Affine3d transformMat =
      tf2::transformToEigen(mRadarTransformStamped.transform).inverse();
  Eigen::Matrix3d rotationMat = transformMat.matrix().block<3, 3>(0, 0);
  Eigen::Vector3d translationVec = transformMat.matrix().block<3, 1>(0, 3);

  mAdjointMat.block<3, 3>(0, 0) = rotationMat;
  mAdjointMat.block<3, 3>(3, 3) = rotationMat;

  Eigen::Matrix3d skewSymmetricMat;
  skewSymmetricMat << 0, -translationVec(2), translationVec(1),
      translationVec(2), 0, -translationVec(0), -translationVec(1),
      translationVec(0), 0;
  mAdjointMat.block<3, 3>(3, 0) = skewSymmetricMat * rotationMat;

  mRadarToImuRotationMat =
      tf2::transformToEigen(mRadarToImuTransformStamped.transform)
          .matrix()
          .block<3, 3>(0, 0);
}

geometry_msgs::TransformStamped
MotionSegment4D::GetTransformStamped(std::string targetFrame,
                                     std::string sourceFrame) {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  while (!tfBuffer.canTransform(targetFrame, sourceFrame, ros::Time(0),
                                ros::Duration(10)))
    ROS_WARN("Wait for transform from %s to %s", targetFrame.c_str(),
             sourceFrame.c_str());

  transformStamped =
      tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
  return transformStamped;
}

void MotionSegment4D::GetParameter() {
  mNh.getParam("mInputTopic", mInputTopic);
  mNh.getParam("mVelocityThreshold", mVelocityThreshold);
  mNh.getParam("mTimeOffset", mTimeOffset);
}

Eigen::MatrixXd MotionSegment4D::ExtractCols(Eigen::MatrixXd mat,
                                             std::vector<int> cols) {
  Eigen::MatrixXd rm_mat(cols.size(), mat.cols());
  for (int i = 0; i < rm_mat.rows(); i++)
    rm_mat.row(i) = mat.row(cols[i]);
  return rm_mat;
}

void MotionSegment4D::VehicleStateCallback(const geometry_msgs::TwistStamped &msg) {
  mVehicleSpeed = msg.twist;
  mGetVehicleState = true;
};

Eigen::Vector3d MotionSegment4D::VehicleStateToRadarTwist(
    const geometry_msgs::Twist &vehicleSpeed) {
  Eigen::Vector3d vehicleTwist;
  vehicleTwist << vehicleSpeed.linear.x, vehicleSpeed.linear.y, vehicleSpeed.linear.z;

  return mAdjointMat * vehicleTwist;
}

geometry_msgs::TwistWithCovariance
MotionSegment4D::GetEgoMat(const std::vector<RadarTarget> &targets,
                           const geometry_msgs::Twist &vehicleSpeed) {
  ceres::Problem problem;
  Eigen::Vector3d radarTwist = VehicleStateToRadarTwist(vehicleSpeed);
  double vx = radarTwist(0);
  double vy = radarTwist(1);
  double vz = radarTwist(2);

  for (uint i = 0; i < targets.size(); i++) {
    const auto &target = targets[i];
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual(target.azimuthAngle, target.elevationAngle,
                                 -target.rangeRate)),
        new ceres::CauchyLoss(1), &vx, &vy, &vz);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 20;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double std = sqrt(summary.final_cost / summary.num_residuals);

  geometry_msgs::TwistWithCovariance egoVelocity;
  egoVelocity.twist.linear.x = vx;
  egoVelocity.twist.linear.y = vy;
  egoVelocity.twist.linear.z = vz;
  egoVelocity.covariance[0] = std;
  egoVelocity.covariance[7] = std;
  egoVelocity.covariance[14] = std;
  return egoVelocity;
}

std::vector<int> MotionSegment4D::GetStationaryIndex(
    const Eigen::MatrixXd &velocityMat, const Eigen::MatrixXd &directionMat,
    const Eigen::MatrixXd &egoVelocityMat, const double threshold) {
  Eigen::MatrixXd deltaVelocityMat =
      directionMat * egoVelocityMat - velocityMat;
  std::vector<int> stationaryIndex;
  for (int i = 0; i < deltaVelocityMat.rows(); i++) {
    if (fabs(deltaVelocityMat(i, 0)) < threshold) {
      stationaryIndex.push_back(i);
    }
  }
  return stationaryIndex;
}

bool MotionSegment4D::CheckRadarTime(const uint64_t &radarTimeStamp,
                                     const ros::Time &msgTimeStamp) const {
  static constexpr double maxAllowedTimeDiffSeconds = 1000.0;
  static constexpr double nanosecondsToSeconds = 1e-9;

  const double timeDiffSeconds = std::abs(
      (ros::Time(radarTimeStamp * nanosecondsToSeconds) - msgTimeStamp)
          .toSec());

  if (timeDiffSeconds > maxAllowedTimeDiffSeconds) {
    ROS_WARN_STREAM(mInputTopic << " is not synced, using header time. "
                                << "Time difference: " << timeDiffSeconds
                                << " seconds");
    return false;
  }
  return true;
}
} // namespace MS4D
