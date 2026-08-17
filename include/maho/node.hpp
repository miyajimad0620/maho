#ifndef MAHO__NODE_HPP_
#define MAHO__NODE_HPP_

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
  Pose2D pose;
  Twist2D twist;
};

#endif  // MAHO__NODE_HPP_
