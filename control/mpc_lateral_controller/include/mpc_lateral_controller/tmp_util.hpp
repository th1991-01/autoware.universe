

namespace motion_utils {

size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
  {

  }

bool isDrivingForward(mpc_traj_resampled.toTrajectoryPoints()){

}
}

namespace tier4_autoware_utils {

double calcCurvature(p1, p2, p3) {

}

double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}


double normalizeRadian(const double rad, const double min_rad = -pi)
{
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}



geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform transform;
  transform.translation = createTranslation(x, y, z);
  transform.rotation = createQuaternion(0.0, 0.0, 0.0, 1.0);
  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(p, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, pose);
  return pose;
}


}