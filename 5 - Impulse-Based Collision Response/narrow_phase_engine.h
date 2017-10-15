#pragma once
#include "Antons_maths_funcs.h"

class NarrowPhaseEngine {
public:
	inline float point_to_point_dist(vec3 p, vec3 p2);
	float point_to_line_dist(vec3 p, vec3 start, vec3 end, vec3& closest);
	float point_to_plane_dist(vec3 p, vec3 normal, float plane_dist, vec3 closest);

	vec3 point_to_line_closest_point(vec3 p, vec3 start, vec3 end);
	vec3 point_to_plane_closest_point(vec3 p, vec3 normal, vec3 plane_point);

	vec3 point_to_triangle(vec3 p, vec3 a, vec3 b, vec3 c, int &region);

	float edge_to_edge(vec3 start_a, vec3 end_a, vec3 start_b, vec3 end_b, vec3& closest_a, vec3& closest_b);
	inline float clamp(float n, float min, float max);

	float triangle_to_triangle(vec3 triangle_a[], vec3 triangle_b[], vec3& closest_a, vec3& closest_b, int region[]);
};