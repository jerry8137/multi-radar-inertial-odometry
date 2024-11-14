#include <motion_segment_4d/motion_segment_4d.h>

namespace MS4D {

void MotionSegment4D::ContiCallback(const sensor_msgs::PointCloud2 &msg) {
  Eigen::MatrixXd velocityMat(int(msg.width), 1);
  Eigen::MatrixXd directionMat(int(msg.width), 3);
  PointCloud::Ptr cloud(new PointCloud);

  const auto targets = reinterpret_cast<const RadarTarget *>(msg.data.data());
  const uint targetsNum = msg.data.size() / sizeof(RadarTarget);
  std::vector<RadarTarget> targetsVec(targets, targets + targetsNum);

  if (!mUseRadarTime.has_value()) {
    // check if time in radar data is synchronized with ros time
    mUseRadarTime = CheckRadarTime(targets[0].t, msg.header.stamp);
  }

  const uint64_t radarTimeStamp =
      mUseRadarTime.value() ? targets[0].t : msg.header.stamp.toNSec();

  for (auto i = decltype(targetsNum){}; i < targetsNum; i++) {
    auto &target = targets[i];
    Point p;
    p.x = target.x;
    p.y = target.y;
    p.z = target.z;
    p.intensity = target.rcs;
    cloud->points.push_back(p);

    velocityMat(i, 0) = -target.rangeRate;
    directionMat(i, 0) = cos(target.azimuthAngle) * cos(target.elevationAngle);
    directionMat(i, 1) = sin(target.azimuthAngle) * cos(target.elevationAngle);
    directionMat(i, 2) = sin(target.elevationAngle);
  }

  geometry_msgs::TwistWithCovariance egoVelocity =
      GetEgoMat(targetsVec, mVehicleSpeed);
  Eigen::Vector3d egoVelocityMat =
      Eigen::Vector3d(egoVelocity.twist.linear.x, egoVelocity.twist.linear.y,
                      egoVelocity.twist.linear.z);
  mLastEgoVelocity = egoVelocityMat;

  std::vector<int> stationaryIndex = GetStationaryIndex(
      velocityMat, directionMat, egoVelocityMat, mVelocityThreshold);

  Eigen::MatrixXd deltaVelocityMat =
      directionMat * egoVelocityMat - velocityMat;

  PointCloud::Ptr stationaryCloud(new PointCloud);
  PointCloud::Ptr nonStationaryCloud(new PointCloud);

  visualization_msgs::MarkerArray markerArray;

  for (auto i = decltype(targetsNum){}; i < targetsNum; i++) {
    bool stationary = std::find(stationaryIndex.begin(), stationaryIndex.end(),
                                i) != stationaryIndex.end();
    Point p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.intensity = deltaVelocityMat(i, 0);

    if (targets[i].ambiguityFlag < 50)
      continue;
    else if (stationary) {
      stationaryCloud->points.push_back(p);
    } else {
      nonStationaryCloud->points.push_back(p);
      // draw velocity using markerarray arrow
      geometry_msgs::Quaternion quatRotation;
      tf::quaternionTFToMsg(
          tf::createQuaternionFromRPY(0, 0, targets[i].azimuthAngle),
          quatRotation);

      // Arrow marker for velocity
      visualization_msgs::Marker pose;
      pose.header.frame_id = msg.header.frame_id;
      pose.header.stamp = msg.header.stamp;
      pose.ns = "track";
      pose.id = i;
      pose.lifetime = ros::Duration(0.1);
      pose.type = visualization_msgs::Marker::ARROW;
      pose.action = visualization_msgs::Marker::ADD;
      pose.scale.x = deltaVelocityMat(i, 0);
      pose.scale.y = 0.8;
      pose.scale.z = 0.8;
      pose.color.a = 1.f;
      pose.color.r = 1.f;
      pose.color.g = 1.f;
      pose.color.b = 0;
      pose.pose.position.x = cloud->points[i].x;
      pose.pose.position.y = cloud->points[i].y;
      pose.pose.position.z = cloud->points[i].z;
      pose.pose.orientation = quatRotation;
      markerArray.markers.push_back(pose);
    }
  }

  stationaryCloud->header.stamp = radarTimeStamp / 1e3;
  nonStationaryCloud->header.stamp = radarTimeStamp / 1e3;

  stationaryCloud->header.frame_id = msg.header.frame_id;
  nonStationaryCloud->header.frame_id = msg.header.frame_id;

  mRadarCloudPub.publish(stationaryCloud);
  mRadarNonStationaryCloudPub.publish(nonStationaryCloud);
  mTargetVelocityPub.publish(markerArray);

  geometry_msgs::TwistWithCovarianceStamped egoVelocityStamped;
  egoVelocityStamped.header.stamp = ros::Time(radarTimeStamp / 1e9);
  egoVelocityStamped.header.frame_id = "imu";
  Eigen::Vector3d egoVelocityVec = mRadarToImuRotationMat * egoVelocityMat;
  egoVelocityStamped.twist = egoVelocity;
  egoVelocityStamped.twist.twist.linear.x = egoVelocityVec(0);
  egoVelocityStamped.twist.twist.linear.y = egoVelocityVec(1);
  egoVelocityStamped.twist.twist.linear.z = egoVelocityVec(2);
  mEgoVelocityPub.publish(egoVelocityStamped);
}
} // namespace MS4D
