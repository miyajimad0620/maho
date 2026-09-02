#include "maho/maho.hpp"

#include "maho/kinematics.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

constexpr std::size_t kPositionUpdateCount = 50;
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

void PrintNodeSequences(const char* count_label, std::size_t count,
                        const Pose2D& current_pose, const Node& command,
                        double first_dt,
                        const Maho::NodeSequenceStatuses& node_sequences) {
  std::cout << "  - " << count_label << ": " << count
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
    const Maho::NodeSequenceStatus& status = node_sequences[rank];
    std::cout << "      - rank: " << rank << "\n"
              << "        collides: "
              << (status.collides ? "true" : "false") << "\n";
    if (status.nodes.empty()) {
      std::cout << "        nodes: []\n";
      continue;
    }

    std::cout << "        nodes:\n";
    Pose2D pose = current_pose;
    for (std::size_t node_index = 0;
         node_index < status.nodes.size(); ++node_index) {
      const Node& node = status.nodes[node_index];
      pose = IntegratePose(pose, node.velocity,
                           node_index == 0 ? first_dt : kPredictionDt);
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
  std::cout << std::fixed << std::setprecision(2);

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
  const Selector selector({8.0, 0.2}, evaluation_function);
  const Optimizer optimizer({
      0.002,
      1e-3,
      {2.0, 2.0, 1.5},
      {0.2, 0.2, 0.15},
  }, evaluation_function);

  Maho maho(params, expander, collision_detector, selector, optimizer);
  Pose2D current_pose = params.initial_pose;
  const Maho::NodeSequences initial_safe_node_sequences = maho.get_nodes();
  const Maho::NodeSequenceStatuses initial_node_sequences =
      maho.get_node_sequences_with_status();
  Node command = params.initial_node;
  if (!initial_safe_node_sequences.empty() &&
      !initial_safe_node_sequences.front().empty()) {
    command = initial_safe_node_sequences.front().front();
  }

  PrintObstacles(params.env);
  PrintGoal(params.goal);
  std::cout << "initialization_steps:\n";
  const Maho::InitializationHistory& initialization_history =
      maho.get_initialization_history();
  for (std::size_t step = 0; step < initialization_history.size(); ++step) {
    PrintNodeSequences("initialization_step", step, params.initial_pose,
                       params.initial_node, kPredictionDt,
                       initialization_history[step]);
  }
  std::cout << "position_updates:\n";
  PrintNodeSequences("position_update_count", 0, current_pose, command,
                     kPredictionDt,
                     initial_node_sequences);
  double dt_replan = 0.0;
  for (std::size_t update_count = 1;
       update_count <= kPositionUpdateCount; ++update_count) {
    current_pose = IntegratePose(current_pose, command.velocity,
                                 kPositionUpdateDt);
    dt_replan = std::min(kPredictionDt, dt_replan + kPositionUpdateDt);
    maho.update_pose(current_pose, dt_replan);

    if (update_count % kReplanPositionUpdateInterval == 0) {
      maho.replan(current_pose);
      dt_replan = 0.0;
      const Maho::NodeSequences safe_node_sequences = maho.get_nodes();
      if (!safe_node_sequences.empty() &&
          !safe_node_sequences.front().empty()) {
        command = safe_node_sequences.front().front();
      }
    }

    const Maho::NodeSequenceStatuses node_sequences =
        maho.get_node_sequences_with_status();
    PrintNodeSequences("position_update_count", update_count,
                       current_pose, command,
                       kPredictionDt - dt_replan, node_sequences);

    if (maho.is_goal_reached(current_pose, command)) {
      break;
    }
  }
}
