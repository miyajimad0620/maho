#include "maho/kinematics.hpp"

#include <cmath>

Pose2D IntegratePose(const Pose2D& pose, const Twist2D& velocity,
                     double dt) {
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);
  return {
      pose.x +
          (cos_theta * velocity.x - sin_theta * velocity.y) * dt,
      pose.y +
          (sin_theta * velocity.x + cos_theta * velocity.y) * dt,
      pose.theta + velocity.theta * dt,
  };
}

Pose2D CalculateTerminalPose(const Pose2D& initial_pose,
                             const Nodes& nodes, double dt) {
  Pose2D pose = initial_pose;
  for (const Node& node : nodes) {
    pose = IntegratePose(pose, node.velocity, dt);
  }
  return pose;
}
