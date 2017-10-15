#include "rigidBodyEngine.h"

inline void RigidBodyEngine::gravity(RigidBody &body) {
	body.force += gravity_vec*body.mass;
}

inline void RigidBodyEngine::spring(RigidBody &body) {
	vec3 attach_point = body.current_verts[0];
	vec3 spring_vector = vec3(attach_point - body.spring.spring_origin);
	float spring_length = length(spring_vector);
	vec3 spring_dir = normalise(spring_vector);
	vec3 spring_force = spring_dir*-(body.spring.k_spring*(spring_length - body.spring.resting_length) + body.spring.zeta*(dot((body.v + body.w*body.spring_attach_point_r), spring_dir)));
	body.force += spring_force;

	body.torque += cross(attach_point - body.x, spring_force);
}

inline void RigidBodyEngine::faux_drag(RigidBody &body) {
	body.force += body.v*-drag_coeff;
	body.torque += body.w * 2 * -drag_coeff;
}


void RigidBodyEngine::update_body(RigidBody &body, vec3 plane_normal, float plane_dist) {
	accumulate_forces_and_toque(body);
	detect_planar_collisions(plane_normal, plane_dist, body);
	apply_forces_and_torque(body);
}


void RigidBodyEngine::accumulate_forces_and_toque(RigidBody &body) {
	body.force = vec3(0.0, 0.0, 0.0);
	body.torque = vec3(0.0, 0.0, 0.0);

	gravity(body);
	//spring(body);
	faux_drag(body);
}

void RigidBodyEngine::apply_forces_and_torque(RigidBody &body) {
	body.v = body.P*body.one_over_mass;
	body.x += body.v*timestep;
	body.P += body.force*timestep;

	mat3 I_inv = body.R*body.I_body_inv*transpose(body.R);
	body.w = I_inv*body.L;
	body.R = orthonormalise(body.R + mat3(0, body.w.v[2], -body.w.v[1], -body.w.v[2], 0, body.w.v[0], body.w.v[1], -body.w.v[0], 0)*body.R*timestep);
	body.L += body.torque*timestep;
}

void RigidBodyEngine::detect_collisions(std::vector<RigidBody*> bodies, int num) {
	collision_detector.broadPhase(bodies, num);
}

void RigidBodyEngine::detect_planar_collisions(vec3 plane_normal, float plane_dist, RigidBody &body) {
	planar_collider.bodyAndPlaneCollision(plane_normal, plane_dist, body);
}