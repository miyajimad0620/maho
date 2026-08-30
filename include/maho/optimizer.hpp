#ifndef MAHO__OPTIMIZER_HPP_
#define MAHO__OPTIMIZER_HPP_

#include <cstddef>

#include "maho/env.hpp"
#include "maho/evaluation_function.hpp"
#include "maho/node.hpp"

struct OptimizerParams {
  double learning_rate;
  double finite_difference_step;
  Twist2D max_velocity;
  Twist2D max_velocity_change;
};

class Optimizer {
 public:
  Optimizer(const OptimizerParams& params,
            const EvaluationFunction& evaluation_function);

  Nodes optimize(const Nodes& nodes, const Pose2D& initial_pose, double dt,
                 double first_dt, const Env& env,
                 const Pose2D& goal,
                 std::size_t fixed_node_count = 0) const;

 private:
  void enforceVelocityConstraints(Nodes* nodes,
                                  std::size_t fixed_node_count) const;

  OptimizerParams params_;
  EvaluationFunction evaluation_function_;
};

#endif  // MAHO__OPTIMIZER_HPP_
