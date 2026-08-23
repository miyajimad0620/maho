#include "maho/evaluator.hpp"

Evaluator::Evaluator(const EvaluationFunction& evaluation_function)
    : evaluation_function_(evaluation_function) {}

double Evaluator::evaluate(const Nodes& nodes, const Env& env,
                           const Goal& goal) const {
  return evaluation_function_.evaluate(
      nodes, env, goal, CollisionHandling::kReturnInfinity);
}
