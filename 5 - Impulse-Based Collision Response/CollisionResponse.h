#pragma once
#include "rigidBody.h"

class CollisionResponder {
public:
	float epsilon = 0.41f;

	bool planeAndBody(vec3 plane_normal, vec3 point_of_contact, RigidBody &body, float dist_to_plane, float &j) {
		vec3 p_dot_a = body.v + (cross(body.w, (point_of_contact - body.x)));
		float v_neg_rel = dot(plane_normal, p_dot_a);
		float dot_prod = dot(point_of_contact - (plane_normal*-dist_to_plane), plane_normal);
		if (v_neg_rel <= 0.0 || dot_prod <= 0) {
			vec3 r_a = point_of_contact - body.x;

			j = abs((-(1 + epsilon)*v_neg_rel) / (body.one_over_mass + dot(plane_normal, cross(body.I_body_inv*cross(r_a, plane_normal), r_a))));
			body.P += plane_normal*j;

			body.L += cross(r_a, plane_normal*j);

			return true;
		}
		else {
			j = 0;
			return false;
		}
	}
};