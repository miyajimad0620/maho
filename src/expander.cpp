#include "maho/expander.hpp"

#include <algorithm>
#include <cmath>

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
      {std::clamp(-node.twist.x, -params_.velocity_step.x,
                  params_.velocity_step.x),
       std::clamp(-node.twist.y, -params_.velocity_step.y,
                  params_.velocity_step.y),
       std::clamp(-node.twist.theta, -params_.velocity_step.theta,
                  params_.velocity_step.theta)},
  }};

  for (std::size_t i = 0; i < expanded_nodes.size(); ++i) {
    Node next = node;
    next.twist.x += velocity_deltas[i].x;
    next.twist.y += velocity_deltas[i].y;
    next.twist.theta += velocity_deltas[i].theta;

    const double cos_theta = std::cos(node.pose.theta);
    const double sin_theta = std::sin(node.pose.theta);
    next.pose.x +=
        (cos_theta * next.twist.x - sin_theta * next.twist.y) * params_.dt;
    next.pose.y +=
        (sin_theta * next.twist.x + cos_theta * next.twist.y) * params_.dt;
    next.pose.theta += next.twist.theta * params_.dt;
    expanded_nodes[i] = next;
  }

  return expanded_nodes;
}
