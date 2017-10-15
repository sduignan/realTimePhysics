#pragma once
#include "Antons_maths_funcs.h"
#include <GL/glew.h>
#include "obj_parser.h"
#include <vector>
#include "spring.h"

class RigidBody {
public:
	float mass;
	float one_over_mass;
	mat3 I_body_inv;	//inertial tensor
	vec3 x;	//position (of c.o.m.)
	mat3 R;	//orientation
	vec3 P;	//linear momentum
	vec3 L;	//angular momentum

	vec3 v;
	vec3 w;

	vec3 force;
	vec3 torque;

	vec3* original_vertices;
	vec3* current_vertices;
	vec3* original_normals;
	vec3* current_normals;
	int vertex_count;

	//force variables
	Spring spring;
	float spring_attach_point_r;

	/*------Collision Detection------*/
	//bounding sphere:
	vec3 centre;
	float radius;
	bool colliding = false;
	//OBB:
	vec3 OBB_position;
	mat3 OBB_original_orientation;
	mat3 OBB_orientation;
	vec3 OBB_extents;
	bool OBB_colliding = false;

	//for drawing
	GLuint vao;
	GLuint vertices_vbo;
	GLuint normals_vbo;

	GLuint spring_vao;
	GLuint spring_vbo;
	GLuint string_colour_loc;

	float timestep;

	RigidBody();
	~RigidBody();
	
	void set_mesh(char* filename);
	void set_spring_vao(GLuint colour_loc);
	void update_vertices();
	void draw();
	void draw_spring();
	void reset();

	void cuboid_setup(float h, float w, float d, mat3 init_orientation);

	mat4 get_OBB_orientation();

private:
	inline mat3 cuboid_inertial_tensor(float m, float h, float w, float d);
};

