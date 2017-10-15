#include "rigidBody.h"

RigidBody::RigidBody() {
	mass = 10.0;
	one_over_mass = 1.0 / mass;

	I_body_inv = cuboid_inertial_tensor(mass, 1.0, 1.0, 1.0);	//cube, side length = 1.0
	x = vec3(0.0,-1.0, 0.0);
	R = identity_mat3();
	P = vec3(0.0, 0.0, 0.0);
	L = vec3(0.0, 0.0, 0.0);

	v = vec3(0.0, 0.0, 0.0);
	w = vec3(0.0, 0.0, 0.0);

	timestep = 1.0 / 60.0;

	force = vec3(0.0, 0.0, 0.0);
	torque = vec3(0.0, 0.0, 0.0);

	//Spring setup
	spring = Spring(10, 0.5, vec3(0.0, 3.0, 0.0), 1.0, 0);
}

inline mat3 RigidBody::cuboid_inertial_tensor(float m, float h, float w, float d) {
	return inverse(mat3((m / 12.0)*((h*h) + (d*d)), 0, 0, 0, (m / 12.0)*((w*w) + (d*d)), 0, 0, 0, (m / 12.0)*((w*w) + (h*h))));
}

void RigidBody::set_mesh(char* filename) {
	GLfloat* vp = NULL; // array of vertex points
	GLfloat* vn = NULL; // array of vertex normals
	GLfloat* vt = NULL; // array of texture coordinates

	load_obj_file(filename, vp, vt, vn, vertex_count);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vertices_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*vertex_count, vp, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenBuffers(1, &normals_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*vertex_count, vn, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, NULL);

	original_vertices = new vec3[vertex_count];
	current_vertices = new vec3[vertex_count];

	for (int i = 0; i < vertex_count; i++) {
		original_vertices[i] = vec3(vp[i * 3], vp[(i * 3) + 1], vp[(i*3)+2]);
		current_vertices[i] = original_vertices[i];
	}

	original_normals = new vec3[vertex_count];
	current_normals = new vec3[vertex_count];

	for (int i = 0; i < vertex_count; i++) {
		original_normals[i] = vec3(vn[i * 3], vn[(i * 3) + 1], vn[(i * 3) + 2]);
		current_normals[i] = original_normals[i];
	}
	spring_attach_point_r = length(original_vertices[0]);	//radius of rotation at attachment point for spring
}

void RigidBody::set_spring_vao() {
	GLfloat vp[] = { spring.spring_origin.v[0], spring.spring_origin.v[1], spring.spring_origin.v[2], current_vertices[0].v[0], current_vertices[0].v[1], current_vertices[0].v[2] }; // array of vertex pointsGLfloat*
	
	glGenVertexArrays(1, &spring_vao);
	glBindVertexArray(spring_vao);

	glGenBuffers(1, &spring_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, spring_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*2, vp, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
}

void RigidBody::update_vertices() {
	float* vert = new float[3 * vertex_count];

	for (int i = 0; i < vertex_count; i++) {
		current_vertices[i] = x + (R*original_vertices[i]);
		vert[i * 3] = current_vertices[i].v[0];
		vert[(i * 3) + 1] = current_vertices[i].v[1];
		vert[(i * 3) + 2] = current_vertices[i].v[2];
	}

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*vertex_count, vert);

	for (int i = 0; i < vertex_count; i++) {
		current_normals[i] = x + (R*original_normals[i]);
		vert[i * 3] = current_normals[i].v[0];
		vert[(i * 3) + 1] = current_normals[i].v[1];
		vert[(i * 3) + 2] = current_normals[i].v[2];
	}

	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*vertex_count, vert);

	delete vert;
}

void RigidBody::draw() {
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, vertex_count);
}

void RigidBody::draw_spring() {
	GLfloat vp[] = { spring.spring_origin.v[0], spring.spring_origin.v[1], spring.spring_origin.v[2], current_vertices[0].v[0], current_vertices[0].v[1], current_vertices[0].v[2] }; // array of vertex pointsGLfloat*
	glBindVertexArray(spring_vao);
	glBindBuffer(GL_ARRAY_BUFFER, spring_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*2, vp);

	glBindVertexArray(spring_vao);
	glDrawArrays(GL_LINES, 0, 2);	
}

RigidBody::~RigidBody() {
	delete original_vertices;
	delete current_vertices;
	delete original_normals;
	delete current_normals;
}

void RigidBody::reset() {
	x = vec3(0.0, -1.0, 0.0);
	R = identity_mat3();
	P = vec3(0.0, 0.0, 0.0);
	L = vec3(0.0, 0.0, 0.0);
	v = vec3(0.0, 0.0, 0.0);
	w = vec3(0.0, 0.0, 0.0);
	force = vec3(0.0, 0.0, 0.0);
	torque = vec3(0.0, 0.0, 0.0);
}