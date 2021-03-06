#include "narrow_phase_engine.h"

inline float NarrowPhaseEngine::point_to_point_dist(vec3 p, vec3 p2) {
	return length(p - p2);
}

float NarrowPhaseEngine::point_to_line_dist(vec3 p, vec3 start, vec3 end, vec3& closest) {
	vec3 line = end - start;
	float line_length_sqrd = length2(line);
	vec3 norm_line = normalise(line);
	vec3 addition = norm_line*dot((p - start), norm_line);
	
	float dot_prod = dot(line, addition);
	
	if (dot_prod <= 0) {
		closest = start;
	}
	else if (dot_prod >= line_length_sqrd) {
		closest = end;
	}
	else {
		closest = start + addition;
	}

	return length(closest - p);
}

float NarrowPhaseEngine::point_to_plane_dist(vec3 p, vec3 normal, float plane_dist, vec3 closest) {
	vec3 n = normalise(normal);
	vec3 plane_point = n*-plane_dist;
	float d = dot((p - plane_point), n);

	closest = p - n*d;

	return d;
}

//Assumes end points are not closest points
vec3 NarrowPhaseEngine::point_to_line_closest_point(vec3 p, vec3 start, vec3 end) {
	vec3 line = end - start;
	vec3 point_to_start = p - start;
	vec3 norm_line = normalise(line);
	return start + norm_line*dot((p - start), norm_line);
}

vec3 NarrowPhaseEngine::point_to_plane_closest_point(vec3 p, vec3 normal, vec3 plane_point) {
	vec3 n = normalise(normal);
	float d = dot((p - plane_point), n);

	return p - n*d;
}


vec3 NarrowPhaseEngine::point_to_triangle(vec3 p, vec3 a, vec3 b, vec3 c, int &region) {
	//Vertex a's Voronoi region:
	vec3 a_to_p = p - a;
	vec3 a_to_b = b - a;
	vec3 a_to_c = c - a;

	float a_to_p_dot_a_to_b = dot(a_to_p, a_to_b);
	float a_to_p_dot_a_to_c = dot(a_to_p, a_to_c);

	if ((a_to_p_dot_a_to_b <= 0) && (a_to_p_dot_a_to_c <= 0)) {
		region = 0;
		return a;
	}

	//Vertex b's Voronoi region:
	vec3 b_to_p = p - b;
	vec3 b_to_a = a - b;
	vec3 b_to_c = c - b;

	float b_to_p_dot_b_to_a = dot(b_to_p, b_to_a);
	float b_to_p_dot_b_to_c = dot(b_to_p, b_to_c);

	if ((b_to_p_dot_b_to_a <= 0) && (b_to_p_dot_b_to_c <= 0)) {
		region = 1;
		return b;
	}

	//Vertex c's Voronoi region:
	vec3 c_to_p = p - c;
	vec3 c_to_a = a - c;
	vec3 c_to_b = b - c;

	float c_to_p_dot_c_to_a = dot(c_to_p, c_to_a);
	float c_to_p_dot_c_to_b = dot(c_to_p, c_to_b);

	if ((c_to_p_dot_c_to_a <= 0) && (c_to_p_dot_c_to_b <= 0)) {
		region = 2;
		return c;
	}

	//Edge ab's Vonoroi region:
	if (dot(cross(cross(b_to_c, b_to_a), b_to_a), b_to_p) >= 0 && a_to_p_dot_a_to_b >= 0 && b_to_p_dot_b_to_a >= 0) {
		region = 3;
		return point_to_line_closest_point(p, a, b);

	}
	
	//Edge bc's Voronoi region:
	if (dot(cross(cross(c_to_a, c_to_b), c_to_b), c_to_p) >= 0 && b_to_p_dot_b_to_c >= 0 && c_to_p_dot_c_to_b >= 0) {
		region = 4;
		return point_to_line_closest_point(p, b, c);
	}

	//Edge ca's Voronoi region:
	if (dot(cross(cross(a_to_b, a_to_c), a_to_c), a_to_p) >= 0 && c_to_p_dot_c_to_a >= 0 && a_to_p_dot_a_to_c >= 0) {
		region = 5;
		return point_to_line_closest_point(p, c, a);
	}

	//Must be face's Voronoi region:
	region = 6;
	return point_to_plane_closest_point(p, cross(a_to_b, a_to_c), a);
}

float NarrowPhaseEngine::edge_to_edge(vec3 start_a, vec3 end_a, vec3 start_b, vec3 end_b, vec3& closest_a, vec3& closest_b) {
	vec3 edge_a = end_a - start_a;
	vec3 edge_b = end_b - start_b;
	vec3 start_to_start = start_a - start_b;

	// Distance along each edge to closest point on that edge from starting point
	float length_along_a, length_along_b;

	float length_sqrd_a = dot(edge_a, edge_a);
	float length_sqrd_b = dot(edge_b, edge_b);
	float a_dot_starts = dot(edge_a, start_to_start);
	float b_dot_starts = dot(edge_b, start_to_start);
	float a_dot_b = dot(edge_a, edge_b);
	float denom = length_sqrd_a*length_sqrd_b - a_dot_b*a_dot_b;

	//if edges are paralellel, just choose the start point of a as the closest (any point will do)
	if (denom == 0){
		length_along_a = 0.0f;
	}
	else
	{
		length_along_a = clamp(((a_dot_b*b_dot_starts - a_dot_starts*length_sqrd_b)/denom), 0.0f, 1.0f);
	}
	
	length_along_b = (a_dot_b*length_along_a + b_dot_starts) / length_sqrd_b;

	//Ensure point on b is actually within the bounds of the linesegment that forms edge b
	if (length_along_b < 0.0) {
		length_along_b = 0.0f;
		//Clamp ensures point on a is actually within the bounds of the linesegment that forms edge a
		length_along_a = clamp((-a_dot_starts / length_sqrd_a), 0.0f, 1.0f);
	}
	else if (length_along_b > 1.0f) {
		length_along_b = 1.0f;
		//Clamp for same reason as above
		length_along_a = clamp(((a_dot_b - a_dot_starts) / length_sqrd_a), 0.0f, 1.0f);
	}

	//Calculate closest points
	closest_a = start_a + edge_a*length_along_a;
	closest_b = start_b + edge_b*length_along_b;

	return length(closest_b - closest_a);
}

// No point importing a maths library just for this function.
inline float NarrowPhaseEngine::clamp(float n, float min, float max) {
	if (n < min)
		return min;
	if (n > max)
		return max;
	return n;
}

float NarrowPhaseEngine::triangle_to_triangle(vec3 triangle_a[], vec3 triangle_b[], vec3& closest_a, vec3& closest_b, int region[]) {
	// Brute force checking the closest points for each vertex of each triangle and for each pair of edges from opposing triangles
	// Sufficient to check vertices and edges - either one of these will be closest or triangles are parallel in which case any pair of points will do
	vec3 closest_points[15][2];
	float distances[15];
	int regions[15];

	// Closest points for triangle a's vertices
	for (int i = 0; i < 3; i++) {
		closest_points[i][0] = triangle_a[i];
		closest_points[i][1] = point_to_triangle(triangle_a[i], triangle_b[0], triangle_b[1], triangle_b[2], regions[i]);
		distances[i] = length(closest_points[i][0] - closest_points[i][1]);
	}

	// Closest points for triangle b's vertices
	for (int i = 3; i < 6; i++) {
		closest_points[i][1] = triangle_b[i-3];
		closest_points[i][0] = point_to_triangle(triangle_b[i-3], triangle_a[0], triangle_a[1], triangle_a[2], regions[i]);
		distances[i] = length(closest_points[i][0] - closest_points[i][1]);
	}

	// Closest poitns for each pair of edges
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			distances[6+(i*3)+j] = edge_to_edge(triangle_a[i], triangle_a[(i+1)%3], triangle_b[j], triangle_b[(j+1)%3], closest_points[6+(i*3)+j][0], closest_points[6+(i*3)+j][1]);
			regions[6 + (i * 3) + j] = j + 3;
		}
	}

	float shortest = distances[0];
	int shortest_index = 0;

	// Finding the shortest distance for each of the closest point pairs
	for (int i = 1; i < 15; i++) {
		if (distances[i] < shortest) {
			shortest = distances[i];
			shortest_index = i;
		}
	}

	closest_a = closest_points[shortest_index][0];
	closest_b = closest_points[shortest_index][1];
	region[0] = shortest_index;
	region[1] = regions[shortest_index];

	return shortest;
}