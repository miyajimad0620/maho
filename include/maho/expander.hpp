#ifndef MAHO__EXPANDER_HPP_
#define MAHO__EXPANDER_HPP_

#include <array>

#include "maho/node.hpp"

struct ExpanderParams {
  Twist2D velocity_step;
};

class Expander {
 public:
  static constexpr std::size_t kNoVelocityChangeIndex = 6;
  static constexpr std::size_t kExpansionCount = 8;
  static constexpr std::size_t kExpansionPathLength = 5;
  using ExpandedPaths = std::array<Nodes, kExpansionCount>;

  explicit Expander(const ExpanderParams& params);

  ExpandedPaths expand(const Nodes& nodes) const;

 private:
  ExpanderParams params_;
};

#endif  // MAHO__EXPANDER_HPP_
