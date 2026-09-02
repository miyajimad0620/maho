#include "maho/selector.hpp"

#include <algorithm>
#include <cmath>

Selector::Selector(const SelectorParams& params,
                   const EvaluationFunction& evaluation_function)
    : params_(params), evaluation_function_(evaluation_function) {}

double Selector::evaluate(const Nodes& nodes,
                          const Pose2D& initial_pose, double dt,
                          double first_dt, const Env& env,
                          const Goal& goal) const {
  return evaluation_function_.evaluate(nodes, initial_pose, dt, first_dt,
                                       env, goal);
}

double Selector::pathDistance(const Nodes& lhs, const Nodes& rhs,
                              const Pose2D& initial_pose, double dt,
                              double first_dt) const {
  const std::size_t node_count = std::min(lhs.size(), rhs.size());
  if (node_count == 0) {
    return 0.0;
  }

  Pose2D lhs_pose = initial_pose;
  Pose2D rhs_pose = initial_pose;
  double squared_distance = 0.0;
  for (std::size_t i = 0; i < node_count; ++i) {
    const double duration = i == 0 ? first_dt : dt;
    lhs_pose = IntegratePose(lhs_pose, lhs[i].velocity, duration);
    rhs_pose = IntegratePose(rhs_pose, rhs[i].velocity, duration);
    const double dx = lhs_pose.x - rhs_pose.x;
    const double dy = lhs_pose.y - rhs_pose.y;
    squared_distance += dx * dx + dy * dy;
  }
  return params_.position_coefficient *
         std::sqrt(squared_distance / static_cast<double>(node_count));
}
