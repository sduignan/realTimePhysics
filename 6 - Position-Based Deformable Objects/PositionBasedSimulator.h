#pragma once
#include "particle.h"
#include "particleSystem.h"
#include <vector>
#include "obj_parser.h"
#include <GL/glew.h>

class PositionBasedBody {
public:
	particleSystem particle_manager;
	particle *vertices;
	vec3* original_shape;
	vec3 original_com;
	vec3* q;
	vec3* p;
	int vertex_count;
	float alpha, beta;

	mat3 Aqq, Apq, R, A;

	int* drawing_order;
	int drawing_count;

	GLuint vao, vertices_vbo, normals_vbo;
	std::vector<int> stitched_points;
	particle *grab_point = NULL;

	PositionBasedBody() {};
	PositionBasedBody(const char* mesh_file, float mass);
	void set_forces();
	void draw();
	inline vec3 com();

	void calculateDeformity();
	void calculateStaticVariables();
	void calculateApq();

	void update();
	void non_deformable_update();
	void stitched_update();
	void stitched_update(vec3 force);

private:
	inline void set_points_normals(float* vp, float* vn);
};
