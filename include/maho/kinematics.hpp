#ifndef MAHO__KINEMATICS_HPP_
#define MAHO__KINEMATICS_HPP_

#include "maho/node.hpp"

Pose2D IntegratePose(const Pose2D& pose, const Twist2D& velocity,
                     double dt);
Pose2D CalculateTerminalPose(const Pose2D& initial_pose,
                             const Nodes& nodes, double dt);

#endif  // MAHO__KINEMATICS_HPP_
