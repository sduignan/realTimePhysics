#pragma once
#include "rigidBody.h"
#include <vector>

class CollisionDetector {
public:
	CollisionDetector() {};

	void broadPhase(std::vector<RigidBody*> bodies, int num);

	std::vector<RigidBody*> boundingSpheres(std::vector<RigidBody*> bodies, int num);

	void obb(std::vector<RigidBody*> bodies);

	bool obb_to_obb(RigidBody a, vec3 a_axes[], RigidBody b,  vec3 b_axes[]);
};
