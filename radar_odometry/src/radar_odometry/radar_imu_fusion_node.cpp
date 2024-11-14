#include "radar_odometry/radar_odometry.h"
#include <backward.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "roboat_loam");
  backward::SignalHandling signalHandling;

  RadarOdometry::IMUPreintegration ImuP;

  RadarOdometry::TransformFusion TF;

  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
