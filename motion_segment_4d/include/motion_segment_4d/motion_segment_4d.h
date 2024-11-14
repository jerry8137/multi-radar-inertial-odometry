#ifndef _MOTION_SEGMENT_4D_H_INCLUDED_
#define _MOTION_SEGMENT_4D_H_INCLUDED_

#include <algorithm>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <iostream>
#include <optional>
#include <queue>
#include <vector>

// ROS dependence including
#include <Eigen/Dense>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// PCL including
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ceres/ceres.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointXYZI Point;

struct RadarTarget {
  uint64_t t;
  float x;
  float y;
  float z;
  float azimuthAngle;      /*  rad  */
  float azimuthAngleStd;   /*  rad  */
  float elevationAngle;    /*  rad  */
  float elevationAngleStd; /*  rad  */
  float range;             /*   m   */
  float rangeStd;          /*   m   */
  float rangeRate;         /*  m/s  */
  float rangeRateStd;      /*  m/s  */
  uint8_t invalidFlags;
  int8_t rcs; /* dBm2  */
  uint16_t measurementID;
  uint8_t positivePredictiveValue; /*   %   */
  uint8_t classification;
  uint8_t multiTargetProbability; /*   %   */
  uint16_t objectID;
  uint8_t ambiguityFlag;
  uint16_t sortIndex;
};

struct VelocityResidual {
  VelocityResidual(double theta, double phi, double vr)
      : theta_(theta), phi_(phi), vr_(vr) {}
  template <typename T>
  bool operator()(const T *const vx, const T *const vy, const T *const vz,
                  T *residual) const {
    residual[0] = vr_ - (vx[0] * cos(theta_) * cos(phi_) +
                         vy[0] * sin(theta_) * cos(phi_) + vz[0] * sin(phi_));
    return true;
  }

private:
  const double theta_;
  const double phi_;
  const double vr_;
};

namespace MS4D {

class MotionSegment4D {
private:
  ros::NodeHandle mNh;
  // Publisher
  ros::Publisher mRadarCloudPub;
  ros::Publisher mRadarNonStationaryCloudPub;
  ros::Publisher mTargetVelocityPub;
  ros::Publisher mEgoVelocityPub;

  // Subsciber
  ros::Subscriber mContiRadarSub;
  ros::Subscriber mVehicleStateSub;

  std::string mPublishFrame, mInputTopic, mOutputTopic;
  geometry_msgs::Twist mVehicleSpeed;
  bool mGetVehicleState;
  std::optional<bool> mUseRadarTime;
  geometry_msgs::TransformStamped mRadarTransformStamped;
  geometry_msgs::TransformStamped mRadarToImuTransformStamped;
  Eigen::MatrixXd mAdjointMat;
  Eigen::Matrix3d mRadarToImuRotationMat;
  Eigen::Vector3d mLastEgoVelocity;

  double mVelocityThreshold;
  double mTimeOffset;

public:
  explicit MotionSegment4D(ros::NodeHandle &nh);
  Eigen::MatrixXd ExtractCols(Eigen::MatrixXd mat, std::vector<int> cols);
  void ContiCallback(const sensor_msgs::PointCloud2 &msg);
  void VehicleStateCallback(const geometry_msgs::TwistStamped &msg);

  Eigen::Vector3d
  VehicleStateToRadarTwist(const geometry_msgs::Twist &vehicleState);

  void GetParameter();
  geometry_msgs::TransformStamped GetTransformStamped(std::string targetFrame,
                                                      std::string sourceFrame);

  geometry_msgs::TwistWithCovariance
  GetEgoMat(const std::vector<RadarTarget> &targets,
            const geometry_msgs::Twist &vehicleState);
  std::vector<int> GetStationaryIndex(const Eigen::MatrixXd &velocityMat,
                                      const Eigen::MatrixXd &directionMat,
                                      const Eigen::MatrixXd &egoVelocityMat,
                                      const double threshold);
  bool CheckRadarTime(const uint64_t &radarTimeStamp,
                      const ros::Time &msgTimeStamp) const;
};
} // namespace MS4D

#endif //_MOTION_SEGMENT_4D_H_INCLUDED_
