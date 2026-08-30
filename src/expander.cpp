#include "maho/expander.hpp"

#include <algorithm>

Expander::Expander(const ExpanderParams& params) : params_(params) {}

Expander::ExpandedNodes Expander::expand(const Node& node) const {
  ExpandedNodes expanded_nodes;
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

  for (std::size_t branch = 0; branch < expanded_nodes.size(); ++branch) {
    expanded_nodes[branch] = node;
    expanded_nodes[branch].velocity.x += velocity_deltas[branch].x;
    expanded_nodes[branch].velocity.y += velocity_deltas[branch].y;
    expanded_nodes[branch].velocity.theta += velocity_deltas[branch].theta;
  }

  return expanded_nodes;
}
