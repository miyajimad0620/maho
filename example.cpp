#include "maho/maho.hpp"

#include "maho/kinematics.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

constexpr std::size_t kPositionUpdateCount = 500;
constexpr std::size_t kReplanPositionUpdateInterval = 10;
constexpr double kPositionUpdateDt = 0.02;
constexpr double kPredictionDt = 0.2;

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

void PrintPose(const Pose2D& pose) {
  std::cout << "{x: ";
  PrintDouble(pose.x);
  std::cout << ", y: ";
  PrintDouble(pose.y);
  std::cout << ", theta: ";
  PrintDouble(pose.theta);
  std::cout << "}";
}

void PrintVelocity(const Twist2D& velocity) {
  std::cout << "{x: ";
  PrintDouble(velocity.x);
  std::cout << ", y: ";
  PrintDouble(velocity.y);
  std::cout << ", theta: ";
  PrintDouble(velocity.theta);
  std::cout << "}";
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
  std::cout << "goal: ";
  PrintPose(goal);
  std::cout << "\n";
}

void PrintNodeSequences(std::size_t position_update_count,
                        const Pose2D& current_pose, const Node& command,
                        const Maho::NodeSequences& node_sequences) {
  std::cout << "  - position_update_count: " << position_update_count
            << "\n"
            << "    current_pose: ";
  PrintPose(current_pose);
  std::cout << "\n"
            << "    command_velocity: ";
  PrintVelocity(command.velocity);
  std::cout << "\n";
  if (node_sequences.empty()) {
    std::cout << "    node_sequences: []\n";
    return;
  }
  std::cout << "    node_sequences:\n";

  for (std::size_t rank = 0; rank < node_sequences.size(); ++rank) {
    std::cout << "      - rank: " << rank << "\n";
    if (node_sequences[rank].empty()) {
      std::cout << "        nodes: []\n";
      continue;
    }

    std::cout << "        nodes:\n";
    Pose2D pose = current_pose;
    for (const Node& node : node_sequences[rank]) {
      pose = IntegratePose(pose, node.velocity, kPredictionDt);
      std::cout << "          - pose: ";
      PrintPose(pose);
      std::cout << "\n"
                << "            velocity: ";
      PrintVelocity(node.velocity);
      std::cout << "\n";
    }
  }
}

}  // namespace

int main() {
  std::cout << std::setprecision(17);

  const MahoParams params{
      {0.0, 0.0, 0.0},
      {{0.0, 0.0, 0.0}},
      // {{2.0, 1.0}, {4.0, -1.0}, {6.0, 1.0}, {5.0, 0.0}},
      {{2.5, 0.1}, {3.0, 0.1}, {3.5, 0.1}, {4.0, 0.1},{4.5, 0.1}, {5.0, 0.1},  },
      {8.0, 0.0, 0.0},
      kPredictionDt,
      2,
      1,
      {0.1, 0.1, 0.1, 0.1},
  };
  const Expander expander(ExpanderParams{{0.2, 0.2, 0.15}});
  const EvaluationFunction evaluation_function(
      {2.5, 0.75, 0.25, 5.0, 0.2, 0.1,
       {20.0, 1.0, 2.0, 1.0, 0.1, 5.0, 1.0, 1.0, 1.0, 1.5}});
  const CollisionDetector collision_detector({0.25});
  const Selector selector({1.0, 0.5, 0.2}, evaluation_function);
  const Optimizer optimizer({
      0.02,
      1e-4,
      {2.0, 2.0, 1.5},
      {0.2, 0.2, 0.15},
  }, evaluation_function);

  Maho maho(params, expander, collision_detector, selector, optimizer);
  Pose2D current_pose = params.initial_pose;
  const Maho::NodeSequences initial_node_sequences = maho.get_nodes();
  Node command = params.initial_node;
  if (!initial_node_sequences.empty() &&
      !initial_node_sequences.front().empty()) {
    command = initial_node_sequences.front().front();
  }

  PrintObstacles(params.env);
  PrintGoal(params.goal);
  std::cout << "position_updates:\n";
  PrintNodeSequences(0, current_pose, command, initial_node_sequences);
  for (std::size_t update_count = 1;
       update_count <= kPositionUpdateCount; ++update_count) {
    current_pose = IntegratePose(current_pose, command.velocity,
                                 kPositionUpdateDt);
    maho.update_pose(current_pose);

    if (update_count % kReplanPositionUpdateInterval == 0) {
      maho.replan(current_pose);
      const Maho::NodeSequences node_sequences = maho.get_nodes();
      if (!node_sequences.empty() && !node_sequences.front().empty()) {
        command = node_sequences.front().front();
      }
      PrintNodeSequences(update_count, current_pose, command,
                         node_sequences);
    }

    if (maho.is_goal_reached(current_pose, command)) {
      break;
    }
  }
}
