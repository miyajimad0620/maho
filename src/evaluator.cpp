#include "maho/evaluator.hpp"

Evaluator::Evaluator(const EvaluationFunction& evaluation_function)
    : evaluation_function_(evaluation_function) {}

double Evaluator::evaluate(const Nodes& nodes, const Pose2D& initial_pose,
                           double dt, const Env& env, const Goal& goal) const {
  return evaluation_function_.evaluate(
      nodes, initial_pose, dt, env, goal,
      CollisionHandling::kReturnInfinity);
}
