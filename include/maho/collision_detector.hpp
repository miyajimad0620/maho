#ifndef MAHO__COLLISION_DETECTOR_HPP_
#define MAHO__COLLISION_DETECTOR_HPP_

#include "maho/env.hpp"
#include "maho/node.hpp"

struct CollisionDetectorParams {
  double robot_radius;
};

class CollisionDetector {
 public:
  explicit CollisionDetector(const CollisionDetectorParams& params);

  bool detectsCollision(const Nodes& nodes, const Pose2D& initial_pose,
                        double dt, const Env& env) const;

 private:
  CollisionDetectorParams params_;
};

#endif  // MAHO__COLLISION_DETECTOR_HPP_
