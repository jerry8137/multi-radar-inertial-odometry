#include "radar_odometry/radar_odometry.h"

namespace RadarOdometry {
TransformFusion::TransformFusion() {
  mMapToOdom.setIdentity();
  try {
    mTfListener.waitForTransform(mapFrame, baselinkFrame, ros::Time(0),
                                 ros::Duration(3.0));
    mTfListener.lookupTransform(mapFrame, baselinkFrame, ros::Time(0),
                                mMapToOdom);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  mSubImuOdometry = nh.subscribe<nav_msgs::Odometry>(
      odomTopic + "_incremental", 20, &TransformFusion::imuOdometryHandler,
      this, ros::TransportHints().tcpNoDelay());

  mPubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 20);
  mPubImuPath = nh.advertise<nav_msgs::Path>(odomTopic + "/path", 1);
}

void TransformFusion::imuOdometryHandler(
    const nav_msgs::Odometry::ConstPtr &odomMsg) {
  std::lock_guard<std::mutex> lock(mMutex);

  Eigen::Affine3d odomToBaselinkEigen = getTransform(odomMsg);
  Eigen::Affine3d mapToBaseLinkEigen = getTransform(mapFrame, baselinkFrame);

  Eigen::Affine3d mapToOdomEigen = mapToBaseLinkEigen * odomToBaselinkEigen.inverse();
  tf::transformEigenToTF(mapToOdomEigen, mMapToOdom);

  // static tf
  static tf::TransformBroadcaster tfBroadcaster;
  tfBroadcaster.sendTransform(tf::StampedTransform(
      mMapToOdom, odomMsg->header.stamp, mapFrame, odometryFrame));

  tf::Transform odomToImu;
  tf::poseMsgToTF(odomMsg->pose.pose, odomToImu);
  tfBroadcaster.sendTransform(
      tf::StampedTransform(odomToImu, odomMsg->header.stamp, odometryFrame,
                           baselinkFrame + "_odom"));

  // publish IMU path
  static nav_msgs::Path imuPath;
  imuPath.header.frame_id = odometryFrame;
  imuPath.header.stamp = odomMsg->header.stamp;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = odomMsg->header.stamp;
  poseStamped.header.frame_id = odometryFrame;
  poseStamped.pose = odomMsg->pose.pose;
  imuPath.poses.push_back(poseStamped);
  mPubImuPath.publish(imuPath);

  if (imuPath.poses.size() > 1000) {
    imuPath.poses.erase(imuPath.poses.begin(), imuPath.poses.begin() + 100);
  }
}
} // namespace RadarOdometry
