#ifndef MAHO__MAHO_HPP_
#define MAHO__MAHO_HPP_

#include <array>
#include <cstddef>

#include "maho/env.hpp"
#include "maho/evaluator.hpp"
#include "maho/expander.hpp"
#include "maho/optimizer.hpp"
#include "maho/selector.hpp"

struct MahoParams {
  Node initial_node;
  Env env;
  Goal goal;
};

class Maho {
 public:
  static constexpr std::size_t kPathCount = 10;
  static constexpr std::size_t kNodeCount = 20;
  using Paths = std::array<Path, kPathCount>;

  Maho(const MahoParams& params, const Expander& expander,
       const Evaluator& evaluator, const Selector& selector,
       const Optimizer& optimizer);

  void update_init_node(const Node& node);
  void replan();
  const Paths& get_paths() const;

 private:
  static constexpr std::size_t kCandidateCount =
      kPathCount * Expander::kExpansionCount;
  using CandidatePaths = std::array<Path, kCandidateCount>;

  CandidatePaths expandPaths(bool advance_path) const;
  void optimizeAndEvaluate(Path* path) const;
  void sortPaths();

  Env env_;
  Goal goal_;
  Expander expander_;
  Evaluator evaluator_;
  Selector selector_;
  Optimizer optimizer_;
  Paths paths_{};
};

#endif  // MAHO__MAHO_HPP_
