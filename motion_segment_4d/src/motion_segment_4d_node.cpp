#include <motion_segment_4d/motion_segment_4d.h>
#include <backward.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_segment_4d_node");
  backward::SignalHandling signalHandling;
  ros::NodeHandle nh("~");
  MS4D::MotionSegment4D motionSegment4D(nh);
  ros::spin();

  return 0;
}
