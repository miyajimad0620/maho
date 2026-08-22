#ifndef MAHO__OPTIMIZER_HPP_
#define MAHO__OPTIMIZER_HPP_

#include <cstddef>
#include <vector>

#include "maho/env.hpp"
#include "maho/node.hpp"

struct OptimizerParams {
  double obstacle_cost_coefficient;
  double obstacle_influence_distance;
  double robot_radius;
  double goal_position_cost_coefficient;
  double goal_angle_cost_coefficient;
  double velocity_change_cost_coefficient;
  double learning_rate;
  double finite_difference_step;
  std::size_t max_iterations;
  double dt;
  Twist2D max_velocity;
  Twist2D max_velocity_change;
};

class Optimizer {
 public:
  explicit Optimizer(const OptimizerParams& params);

  std::vector<Node> optimize(const std::vector<Node>& nodes, const Env& env,
                             const Pose2D& goal) const;

 private:
  double cost(const std::vector<Node>& nodes, const Env& env,
              const Pose2D& goal) const;
  void enforceKinematicConstraints(std::vector<Node>* nodes) const;

  OptimizerParams params_;
};

#endif  // MAHO__OPTIMIZER_HPP_
