#include "maho/expander.hpp"

#include <algorithm>

Expander::Expander(const ExpanderParams& params) : params_(params) {}

Expander::ExpandedPaths Expander::expand(const Node& node) const {
  ExpandedPaths expanded_paths;
  const std::array<Twist2D, kExpansionCount> velocity_deltas{{
      {-params_.velocity_step.x, 0.0, 0.0},
      {params_.velocity_step.x, 0.0, 0.0},
      {0.0, -params_.velocity_step.y, 0.0},
      {0.0, params_.velocity_step.y, 0.0},
      {0.0, 0.0, -params_.velocity_step.theta},
      {0.0, 0.0, params_.velocity_step.theta},
      {0.0, 0.0, 0.0},
      {std::clamp(-node.velocity.x, -params_.velocity_step.x,
                  params_.velocity_step.x),
       std::clamp(-node.velocity.y, -params_.velocity_step.y,
                  params_.velocity_step.y),
       std::clamp(-node.velocity.theta, -params_.velocity_step.theta,
                  params_.velocity_step.theta)},
  }};

  for (std::size_t i = 0; i < expanded_paths.size(); ++i) {
    Node next = node;
    next.velocity.x += velocity_deltas[i].x;
    next.velocity.y += velocity_deltas[i].y;
    next.velocity.theta += velocity_deltas[i].theta;
    expanded_paths[i].fill(next);
  }

  return expanded_paths;
}
