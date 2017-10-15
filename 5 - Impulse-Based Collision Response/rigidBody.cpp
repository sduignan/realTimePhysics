#include "rigidBody.h"

RigidBody::RigidBody() {
	mass = 10.0;
	one_over_mass = 1.0 / mass;

	//I_body_inv = cuboid_inertial_tensor(mass, 1.0, 1.0, 1.0);	//cube, side length = 1.0
	x = vec3(0.0, -1.0, 0.0);
	R = identity_mat3();
	P = vec3(0.0, 0.0, 0.0);
	L = vec3(0.0, 0.0, 0.0);

	v = vec3(0.0, 0.0, 0.0);
	w = vec3(0.0, 0.0, 0.0);

	cuboid_setup(1.0, 1.0, 1.0, identity_mat3());

	timestep = 1.0 / 60.0;

	force = vec3(0.0, 0.0, 0.0);
	torque = vec3(0.0, 0.0, 0.0);

	//Spring setup
	spring = Spring(10, 0.5, vec3(0.0, 3.0, 0.0), 1.0, 0);
}

inline mat3 RigidBody::cuboid_inertial_tensor(float m, float h, float w, float d) {
	return inverse(mat3((m / 12.0)*((h*h) + (d*d)), 0, 0, 0, (m / 12.0)*((w*w) + (d*d)), 0, 0, 0, (m / 12.0)*((w*w) + (h*h))));
}

void RigidBody::cuboid_setup(float h, float w, float d, mat3 init_orientation) {
	I_body_inv = cuboid_inertial_tensor(mass, h, w, d);

	OBB_orientation = init_orientation;
	OBB_original_orientation = init_orientation;
	OBB_position = x;
	OBB_extents = vec3(w / 2.0, h / 2.0, d / 2.0);
}

inline mat3 RigidBody::octahedron_inertial_tensor(float m, float s) {
	float oneD_moment = m*s*s / 10.0;
	return inverse(mat3(oneD_moment, 0, 0, 0, oneD_moment, 0, 0, 0, oneD_moment));
}

void RigidBody::octahedron_setup(float m, float s, float ext, mat3 init_orientation) {
	mass = m;
	one_over_mass = 1.0 / m;
	I_body_inv = octahedron_inertial_tensor(m, s);

	OBB_orientation = init_orientation;
	OBB_original_orientation = init_orientation;
	OBB_position = x;
	OBB_extents = vec3(ext, ext, ext);
}

void RigidBody::set_mesh(char* filename) {
	GLfloat* vp = NULL; // array of vertex points
	GLfloat* vn = NULL; // array of vertex normals
	GLfloat* vt = NULL; // array of texture coordinates

	load_obj_file(filename, vp, vt, vn, vertex_count);

	//ask GL to generate a VAO, and give us a handle to refer to it with
	glGenVertexArrays(1, &vao);
	// "bind" the VAO
	glBindVertexArray(vao);

	// ask GL to generate a VBO, and give us a handle to refer to it with
	glGenBuffers(1, &vertices_vbo);
	// "bind" the VBO. all future VBO operations affect this VBO until we bind a different one
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	// copy our points from RAM into our VBO on graphics hardware
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*vertex_count, vp, GL_STATIC_DRAW);
	// tell the VAO that it should enable the first shader input variable (number 0)
	glEnableVertexAttribArray(0);
	// tell the VAO that it should use our VBO to find its first input variable (number 0).
	//that input variable should use 3 consecutive floats as a variable (it will be a vec3)
	//GL_FALSE means we dont need to normalise it (useful for some clever data tricks in advanced stuff)
	//each variable comes one-after-the-other in the data, so there's no need for stride (0), and starts at
	//the beginning of the buffer, so no offset either (NULL) 
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenBuffers(1, &normals_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*vertex_count, vn, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, NULL);

	GLuint tex_vbo;
	glGenBuffers(1, &tex_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, tex_vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(float)*vertex_count, vt, GL_STATIC_DRAW);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 0, NULL);

	for (int i = 0; i < vertex_count; i++) {
		vec3 vert(vp[i * 3], vp[(i * 3) + 1], vp[(i * 3) + 2]);
		if (!(std::find(original_verts.begin(), original_verts.end(), vert) != original_verts.end())) {
			unique_vert_indices.push_back(i);
		}
		original_verts.push_back(vert);
		current_verts.push_back(vert);
	}
	
	for (int i = 0; i < vertex_count; i++) {
		original_norms.push_back(vec3(vn[i * 3], vn[(i * 3) + 1], vn[(i * 3) + 2]));
		current_norms.push_back(vec3(vn[i * 3], vn[(i * 3) + 1], vn[(i * 3) + 2]));
	}
	spring_attach_point_r = length(original_verts[0]);	//radius of rotation at attachment point for spring

	centre = vec3(0, 0, 0);

	radius = 0;
	for (int i = 0; i < vertex_count; i++) {
		for (int j = i + 1; j < vertex_count; j++) {
			float len = length2(original_verts[i] - original_verts[j]);
			if (len > radius) {
				radius = len;
			}
		}
	}
	radius = sqrt(radius)/2.0;
}

void RigidBody::set_spring_vao(GLuint colour_loc) {
	GLfloat vp[] = { spring.spring_origin.v[0], spring.spring_origin.v[1], spring.spring_origin.v[2], current_verts[0].v[0], current_verts[0].v[1], current_verts[0].v[2] }; // array of vertex pointsGLfloat*

	//ask GL to generate a VAO, and give us a handle to refer to it with
	glGenVertexArrays(1, &spring_vao);
	// "bind" the VAO
	glBindVertexArray(spring_vao);

	// ask GL to generate a VBO, and give us a handle to refer to it with
	glGenBuffers(1, &spring_vbo);
	// "bind" the VBO. all future VBO operations affect this VBO until we bind a different one
	glBindBuffer(GL_ARRAY_BUFFER, spring_vbo);
	// copy our points from RAM into our VBO on graphics hardware
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*2, vp, GL_STATIC_DRAW);
	// tell the VAO that it should enable the first shader input variable (number 0)
	glEnableVertexAttribArray(0);
	// tell the VAO that it should use our VBO to find its first input variable (number 0).
	//that input variable should use 3 consecutive floats as a variable (it will be a vec3)
	//GL_FALSE means we dont need to normalise it (useful for some clever data tricks in advanced stuff)
	//each variable comes one-after-the-other in the data, so there's no need for stride (0), and starts at
	//the beginning of the buffer, so no offset either (NULL) 
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	string_colour_loc = colour_loc;
}

void RigidBody::update_vertices() {
	float* vert = new float[3 * vertex_count];

	for (int i = 0; i < vertex_count; i++) {
		current_verts[i] = x + (R*original_verts[i]);
		
		vert[i * 3] = current_verts[i].v[0];
		vert[(i * 3) + 1] = current_verts[i].v[1];
		vert[(i * 3) + 2] = current_verts[i].v[2];
	}


	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*vertex_count, vert);

	for (int i = 0; i < vertex_count; i++) {
		current_norms[i] = x + (R*original_norms[i]);
		vert[i * 3] = current_norms[i].v[0];
		vert[(i * 3) + 1] = current_norms[i].v[1];
		vert[(i * 3) + 2] = current_norms[i].v[2];
	}

	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*vertex_count, vert);

	delete vert;

	centre = x;
	OBB_position = x;
	OBB_orientation = R*OBB_original_orientation;
}

void RigidBody::draw() {
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, vertex_count);
}

void RigidBody::draw_spring() {
	GLfloat vp[] = { spring.spring_origin.v[0], spring.spring_origin.v[1], spring.spring_origin.v[2], current_verts[0].v[0], current_verts[0].v[1], current_verts[0].v[2] }; // array of vertex pointsGLfloat*
	glBindVertexArray(spring_vao);
	glBindBuffer(GL_ARRAY_BUFFER, spring_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*2, vp);

	glBindVertexArray(spring_vao);
	glUniform3fv(string_colour_loc, 1, vec3(0.0, 0.0, 0.0).v);
	glDrawArrays(GL_LINES, 0, 2);	
}

RigidBody::~RigidBody() {
}

void RigidBody::reset() {
	x = vec3(0.0, 2.5, 0.0);
	R = identity_mat3();
	P = vec3(0.0, 0.0, 0.0);
	L = vec3(0.0, 0.0, 0.0);
	v = vec3(0.0, 0.0, 0.0);
	w = vec3(0.0, 0.0, 0.0);
	force = vec3(0.0, 0.0, 0.0);
	torque = vec3(0.0, 0.0, 0.0);
}


mat4 RigidBody::get_OBB_orientation() {
	mat4 obb_mat4 = identity_mat4();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			obb_mat4.m[i*4 + j] = OBB_orientation.m[i*3 + j];
		}
	}
	return obb_mat4;
}