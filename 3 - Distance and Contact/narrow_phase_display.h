#pragma once
#include "narrow_phase_engine.h"
#include <GL/glew.h>

class NarrowPhaseDisplay {
public:
	vec3 point;
	vec3 triangle[3];
	vec3 triangle_normals[3];
	vec3 triangle_b[3];
	vec3 triangle_b_normals[3];

	GLuint triangle_vao;
	GLuint triangle_vbo;
	GLuint normals_vbo;
	GLuint triangle_b_vao;
	GLuint triangle_b_vbo;
	GLuint triangle_b_normals_vbo;

	GLuint line_vao;
	GLuint line_vbo;

	GLuint triangle_shader_id;
	GLuint line_shader_id;

	int triangle_colour_loc;
	int line_colour_loc;

	NarrowPhaseEngine engine;

	NarrowPhaseDisplay();

	void setup(GLuint tri_shader, GLuint line_shader, int tri_col_loc, int line_col_loc);

	float update_and_draw();
	float find_edge_point_and_draw();
	float t_to_t_update_and_draw();
};