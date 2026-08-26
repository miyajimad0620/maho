#ifndef MAHO__NODE_HPP_
#define MAHO__NODE_HPP_

#include <vector>

struct Twist2D {
  double x;
  double y;
  double theta;
};

struct Pose2D {
  double x;
  double y;
  double theta;
};

struct Node {
  Twist2D velocity;
};

using Nodes = std::vector<Node>;

#endif  // MAHO__NODE_HPP_
