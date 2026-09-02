#ifndef MAHO__TRAJECTORY_HPP_
#define MAHO__TRAJECTORY_HPP_

#include "maho/env.hpp"
#include "maho/node.hpp"

struct Trajectory2D {
  Pose2D initial_pose;
  Pose2D terminal_pose;
  Point2D center;
  double radius;
  double swept_angle;
  bool is_straight;
};

Trajectory2D CalculateTrajectory(const Pose2D& initial_pose,
                                 const Twist2D& velocity, double dt);
double CalculateMinimumDistanceToTrajectory(const Trajectory2D& trajectory,
                                            const Point2D& point);

#endif  // MAHO__TRAJECTORY_HPP_
