#pragma once
#include "rigidBody.h"
#include "spring.h"
#include "collisionDetector.h"


class RigidBodyEngine {
public:
	vec3 gravity_vec = vec3(0.0, -1.0, 0.0);
	float drag_coeff = 0.2;
	float timestep = 1.0 / 60.0;
	CollisionDetector collision_detector;

	RigidBodyEngine() {};
	~RigidBodyEngine() {};

	void update_body(RigidBody &body);


	inline void gravity(RigidBody &body);
	inline void spring(RigidBody &body);
	inline void faux_drag(RigidBody &body);

	void accumulate_forces_and_toque(RigidBody &body);
	void apply_forces_and_torque(RigidBody &body);
	void detect_collisions(std::vector<RigidBody*> bodies, int num);
};