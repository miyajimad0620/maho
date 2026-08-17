#ifndef MAHO__EXPANDER_HPP_
#define MAHO__EXPANDER_HPP_

#include <array>

#include "maho/node.hpp"

struct ExpanderParams {
  double dt;
  Twist2D velocity_step;
};

class Expander {
 public:
  static constexpr std::size_t kExpansionCount = 7;
  using ExpandedNodes = std::array<Node, kExpansionCount>;

  explicit Expander(const ExpanderParams& params);

  ExpandedNodes expand(const Node& node) const;

 private:
  ExpanderParams params_;
};

#endif  // MAHO__EXPANDER_HPP_
