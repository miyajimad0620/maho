#include "maho/maho.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

constexpr std::size_t kSimulationStepCount = 50;
constexpr double kDt = 0.2;

void AdvanceNode(Node* node) {
  const double cos_theta = std::cos(node->pose.theta);
  const double sin_theta = std::sin(node->pose.theta);
  node->pose.x +=
      (cos_theta * node->twist.x - sin_theta * node->twist.y) * kDt;
  node->pose.y +=
      (sin_theta * node->twist.x + cos_theta * node->twist.y) * kDt;
  node->pose.theta += node->twist.theta * kDt;
}

void PrintDouble(double value) {
  if (std::isnan(value)) {
    std::cout << ".nan";
  } else if (value == std::numeric_limits<double>::infinity()) {
    std::cout << ".inf";
  } else if (value == -std::numeric_limits<double>::infinity()) {
    std::cout << "-.inf";
  } else {
    std::cout << value;
  }
}

void PrintNode(const Node& node) {
  std::cout << "          - pose: {x: ";
  PrintDouble(node.pose.x);
  std::cout << ", y: ";
  PrintDouble(node.pose.y);
  std::cout << ", theta: ";
  PrintDouble(node.pose.theta);
  std::cout << "}\n"
            << "            twist: {x: ";
  PrintDouble(node.twist.x);
  std::cout << ", y: ";
  PrintDouble(node.twist.y);
  std::cout << ", theta: ";
  PrintDouble(node.twist.theta);
  std::cout << "}\n";
}

void PrintObstacles(const Env& env) {
  if (env.empty()) {
    std::cout << "obstacles: []\n";
    return;
  }

  std::cout << "obstacles:\n";
  for (const Point2D& point : env) {
    std::cout << "  - {x: ";
    PrintDouble(point.x);
    std::cout << ", y: ";
    PrintDouble(point.y);
    std::cout << "}\n";
  }
}

void PrintGoal(const Goal& goal) {
  std::cout << "goal: {x: ";
  PrintDouble(goal.x);
  std::cout << ", y: ";
  PrintDouble(goal.y);
  std::cout << ", theta: ";
  PrintDouble(goal.theta);
  std::cout << "}\n";
}

void PrintPaths(std::size_t step, const Node& current_node,
                const Maho::Paths& paths) {
  std::cout << "  - step: " << step << "\n"
            << "    current_node:\n"
            << "      pose: {x: ";
  PrintDouble(current_node.pose.x);
  std::cout << ", y: ";
  PrintDouble(current_node.pose.y);
  std::cout << ", theta: ";
  PrintDouble(current_node.pose.theta);
  std::cout << "}\n"
            << "      twist: {x: ";
  PrintDouble(current_node.twist.x);
  std::cout << ", y: ";
  PrintDouble(current_node.twist.y);
  std::cout << ", theta: ";
  PrintDouble(current_node.twist.theta);
  std::cout << "}\n"
            << "    paths:\n";
  for (std::size_t rank = 0; rank < paths.size(); ++rank) {
    const Path& path = paths[rank];
    std::cout << "      - rank: " << rank << "\n"
              << "        cost: ";
    PrintDouble(path.cost);
    if (path.nodes.empty()) {
      std::cout << "\n"
                << "        nodes: []\n";
      continue;
    }

    std::cout << "\n"
              << "        nodes:\n";
    for (const Node& node : path.nodes) {
      PrintNode(node);
    }
  }
}

}  // namespace

int main() {
  std::cout << std::setprecision(17);

  const MahoParams params{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{2.0, 1.0}, {4.0, -1.0}, {6.0, 1.0}},
      {8.0, 0.0, 0.0},
  };
  const Expander expander({kDt, {0.2, 0.2, 0.15}});
  const Evaluator evaluator({0.1, 0.25, 1.0});
  const Selector selector({1.0, 0.5, 0.2});
  const Optimizer optimizer({
      0.1,
      0.75,
      0.25,
      5.0,
      0.2,
      0.1,
      0.02,
      1e-4,
      2,
      kDt,
      {2.0, 2.0, 1.5},
      {0.2, 0.2, 0.15},
  });

  Maho maho(params, expander, evaluator, selector, optimizer);
  Node current_node = params.initial_node;

  PrintObstacles(params.env);
  PrintGoal(params.goal);
  std::cout << "steps:\n";
  PrintPaths(0, current_node, maho.get_paths());
  for (std::size_t step = 1; step <= kSimulationStepCount; ++step) {
    current_node.twist = maho.get_paths().front().nodes[1].twist;
    AdvanceNode(&current_node);
    maho.replan();
    maho.update_init_node(current_node);
    PrintPaths(step, current_node, maho.get_paths());
  }
}
