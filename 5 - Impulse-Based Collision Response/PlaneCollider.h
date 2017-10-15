#pragma once
#include "CollisionResponse.h"
#include "narrow_phase_engine.h"
#include <vector>

class PlaneCollider {
public:
	NarrowPhaseEngine dist_checker;
	CollisionResponder collider;
	float allowance = 0.01f;


	bool checkBoundingSphere(vec3 plane_normal, float plane_dist, RigidBody body) {
		vec3  closest;
		float dist = dist_checker.point_to_plane_dist(body.centre, plane_normal, plane_dist, closest);
		return (dist<(body.radius+allowance));
	};

	void bodyAndPlaneCollision(vec3 plane_normal, float plane_dist, RigidBody &body) {
		if (checkBoundingSphere(plane_normal, plane_dist, body)) {
			std::vector<vec3> closest_vertices;
			vec3 temp;
			float closest_dist = dist_checker.point_to_plane_dist(body.current_verts[body.unique_vert_indices[0]], plane_normal, plane_dist, temp);
			closest_vertices.push_back(body.current_verts[body.unique_vert_indices[0]]);
			for (int i = 1; i < body.unique_vert_indices.size(); i++) {
				float curr_dist = dist_checker.point_to_plane_dist(body.current_verts[body.unique_vert_indices[i]], plane_normal, plane_dist, temp);
				if (curr_dist < closest_dist) {
					closest_vertices.clear();
				}
				if (curr_dist <= closest_dist) {
					closest_vertices.push_back(body.current_verts[body.unique_vert_indices[i]]);
					closest_dist = curr_dist;
				}
			}
			if (closest_dist < allowance) {
				vec3 contact_point(0, 0, 0);
				for (int i = 0; i < closest_vertices.size(); i++) {
					contact_point += closest_vertices[i];
				}
				contact_point = contact_point*(1.0/(float)closest_vertices.size());

				float j;
				if (collider.planeAndBody(plane_normal, contact_point, body, plane_dist, j)) {
					float dot_prod = dot(body.force*body.timestep, plane_normal);

					//Check if the effect of the forces perpendicular and into the plane in one timestep are greater than the impulse
					if (j < -dot_prod) {
						// If so, body is in resting contact, zero out linear momentum and force perpendiluar to the plane.
						body.P -= plane_normal*dot(body.P, plane_normal);
						body.force -= plane_normal*dot_prod;
					}
				}
			}
		}
	};
};