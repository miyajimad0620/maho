#ifndef MAHO__EVALUATOR_HPP_
#define MAHO__EVALUATOR_HPP_

#include "maho/evaluation_function.hpp"

class Evaluator {
 public:
  explicit Evaluator(const EvaluationFunction& evaluation_function);

  double evaluate(const Nodes& nodes, const Env& env, const Goal& goal) const;

 private:
  EvaluationFunction evaluation_function_;
};

#endif  // MAHO__EVALUATOR_HPP_
