#pragma once
#ifndef _UTILITY_RADAR_ODOMETRY_H_
#define _UTILITY_RADAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

typedef pcl::PointXYZI PointType;

class ParamServer {
public:
  ros::NodeHandle nh;

  float smootherLag;

  // Topics
  std::string imuTopic;
  std::string odomTopic;
  std::string velocityOdomTopic;
  std::string rearVelocityOdomTopic;

  // Frames
  std::string lidarFrame;
  std::string baselinkFrame;
  std::string odometryFrame;
  std::string mapFrame;

  float poseCovThreshold;

  bool useRearRadar;
  bool useQRFactorization;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;
  float imuRPYWeight;

  ParamServer() {
    nh.getParam("radar_odometry/imuTopic", imuTopic);
    nh.getParam("radar_odometry/odomTopic", odomTopic);
    nh.getParam("radar_odometry/velocityOdomTopic", velocityOdomTopic);
    if (nh.hasParam("radar_odometry/rearVelocityOdomTopic")) {
      nh.getParam("radar_odometry/rearVelocityOdomTopic", rearVelocityOdomTopic);
      useRearRadar = true;
    }

    nh.getParam("radar_odometry/lidarFrame", lidarFrame);
    nh.getParam("radar_odometry/baselinkFrame", baselinkFrame);
    nh.getParam("radar_odometry/odometryFrame", odometryFrame);
    nh.getParam("radar_odometry/mapFrame", mapFrame);

    nh.getParam("radar_odometry/imuAccNoise", imuAccNoise);
    nh.getParam("radar_odometry/imuGyrNoise", imuGyrNoise);
    nh.getParam("radar_odometry/imuAccBiasN", imuAccBiasN);
    nh.getParam("radar_odometry/imuGyrBiasN", imuGyrBiasN);
    nh.getParam("radar_odometry/imuGravity", imuGravity);
    nh.getParam("radar_odometry/imuRPYWeight", imuRPYWeight);

    nh.getParam("radar_odometry/smootherLag", smootherLag);
    nh.getParam("radar_odometry/useQRFactorization", useQRFactorization);

    usleep(100);
  }
};

template <typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud, ros::Time thisStamp,
                                      std::string thisFrame) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub.getNumSubscribers() != 0)
    thisPub.publish(tempCloud);
  return tempCloud;
}

template <typename T> double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x,
                           T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y,
                       T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch,
                   T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

inline float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

static inline sensor_msgs::Imu
Lerp(const sensor_msgs::Imu &m1, const sensor_msgs::Imu &m2, const double &s) {
  sensor_msgs::Imu result;
  result.angular_velocity.x =
      m1.angular_velocity.x +
      s * (m2.angular_velocity.x - m1.angular_velocity.x);
  result.angular_velocity.y =
      m1.angular_velocity.y +
      s * (m2.angular_velocity.y - m1.angular_velocity.y);
  result.angular_velocity.z =
      m1.angular_velocity.z +
      s * (m2.angular_velocity.z - m1.angular_velocity.z);
  result.linear_acceleration.x =
      m1.linear_acceleration.x +
      s * (m2.linear_acceleration.x - m1.linear_acceleration.x);
  result.linear_acceleration.y =
      m1.linear_acceleration.y +
      s * (m2.linear_acceleration.y - m1.linear_acceleration.y);
  result.linear_acceleration.z =
      m1.linear_acceleration.z +
      s * (m2.linear_acceleration.z - m1.linear_acceleration.z);
  return result;
}

inline Eigen::Affine3d getTransform(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  Eigen::Affine3d transform;
  transform.setIdentity();
  transform.translation() << odomMsg->pose.pose.position.x,
      odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z;
  transform.rotate(Eigen::Quaterniond(
      odomMsg->pose.pose.orientation.w, odomMsg->pose.pose.orientation.x,
      odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z));
  return transform;
}

inline Eigen::Affine3d getTransform(const std::string &frame1,
                             const std::string &frame2) {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  while (!tfBuffer.canTransform(frame1, frame2, ros::Time(0),
                                ros::Duration(1.0))) {
    ROS_WARN("Waiting for transform between %s and %s", frame1.c_str(),
             frame2.c_str());
  }
  geometry_msgs::TransformStamped transform =
      tfBuffer.lookupTransform(frame1, frame2, ros::Time(0));
  Eigen::Affine3d transformEigen;
  transformEigen.setIdentity();
  transformEigen.translation() << transform.transform.translation.x,
      transform.transform.translation.y, transform.transform.translation.z;
  transformEigen.rotate(Eigen::Quaterniond(
      transform.transform.rotation.w, transform.transform.rotation.x,
      transform.transform.rotation.y, transform.transform.rotation.z));
  return transformEigen;
}

#endif
