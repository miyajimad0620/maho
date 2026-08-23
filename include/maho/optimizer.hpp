#ifndef MAHO__OPTIMIZER_HPP_
#define MAHO__OPTIMIZER_HPP_

#include <cstddef>
#include <vector>

#include "maho/env.hpp"
#include "maho/evaluation_function.hpp"
#include "maho/node.hpp"

struct OptimizerParams {
  double learning_rate;
  double finite_difference_step;
  std::size_t max_iterations;
  double dt;
  Twist2D max_velocity;
  Twist2D max_velocity_change;
};

class Optimizer {
 public:
  Optimizer(const OptimizerParams& params,
            const EvaluationFunction& evaluation_function);

  std::vector<Node> optimize(const std::vector<Node>& nodes, const Env& env,
                             const Pose2D& goal) const;

 private:
  void enforceKinematicConstraints(std::vector<Node>* nodes) const;

  OptimizerParams params_;
  EvaluationFunction evaluation_function_;
};

#endif  // MAHO__OPTIMIZER_HPP_
