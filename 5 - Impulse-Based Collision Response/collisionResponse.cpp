#include "CollisionResponse.h"

void CollisionResponder::planeAndBody(vec3 plane_normal, vec3 point_of_contact, RigidBody &body) {
	vec3 p_dot_a = body.v + (cross(body.w, (point_of_contact - body.x)));
	float v_neg_rel = dot(plane_normal, p_dot_a);
	vec3 r_a = point_of_contact - body.x;

	float j = abs((-(1+epsilon)*v_neg_rel)/(body.one_over_mass + dot(plane_normal, cross(body.I_body_inv*cross(r_a, plane_normal), r_a))));
	body.force = plane_normal*j;

	body.torque = cross(r_a, plane_normal*j);
}