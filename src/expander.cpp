#include "maho/expander.hpp"

#include <algorithm>
#include <stdexcept>

Expander::Expander(const ExpanderParams& params) : params_(params) {}

Expander::ExpandedPaths Expander::expand(const Nodes& nodes) const {
  if (nodes.empty()) {
    throw std::invalid_argument("nodes must not be empty");
  }

  ExpandedPaths expanded_paths;
  const Node& node = nodes.back();
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

  for (std::size_t branch = 0; branch < expanded_paths.size(); ++branch) {
    expanded_paths[branch] = nodes;
    Node next = node;
    next.velocity.x += velocity_deltas[branch].x;
    next.velocity.y += velocity_deltas[branch].y;
    next.velocity.theta += velocity_deltas[branch].theta;
    expanded_paths[branch].push_back(next);
    const std::size_t expansion_path_length =
        std::min(kExpansionPathLength, expanded_paths[branch].size());
    std::fill(expanded_paths[branch].end() - expansion_path_length,
              expanded_paths[branch].end(), next);
  }

  return expanded_paths;
}
