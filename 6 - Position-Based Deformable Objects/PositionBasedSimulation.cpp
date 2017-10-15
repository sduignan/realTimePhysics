#include "PositionBasedSimulator.h"
#include "extraMaths.h"

PositionBasedBody::PositionBasedBody(const char* meshFile, float mass) {
	float *points, *vt;

	load_obj_file_as_position_based_body(meshFile, points, vt, vertex_count, drawing_order, drawing_count);
	
	vertices = new particle[vertex_count];
	original_shape = new vec3[vertex_count];

	for (int i = 0; i < vertex_count; i++) {
		original_shape[i] = vec3(points[i * 3], points[(i * 3) + 1], points[(i * 3) + 2]);
		vertices[i].x = original_shape[i];
		vertices[i].x_extreme = original_shape[i];
		if (original_shape[i].v[1] <= -0.2) {
			stitched_points.push_back(i);
		}
		if (original_shape[i].v[1] >= 0.49 && original_shape[i].v[0] >= 0.49 && original_shape[i].v[2] >= 0.49) {
			grab_point = &vertices[i];
		}
		vertices[i].v = vec3(0, 0, 0);
		vertices[i].m = mass / float(vertex_count);
	}

	if (grab_point == NULL) {
		grab_point = &vertices[1];
	}

	particle_manager.setup(vertices, vertex_count, original_shape);

	original_com = com();

	GLfloat *vp = new GLfloat[drawing_count * 3];
	GLfloat *vn = new GLfloat[drawing_count * 3];
	
	set_points_normals(vp, vn);

	// VAO and VBO setups
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vertices_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * drawing_count, vp, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenBuffers(1, &normals_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * drawing_count, vn, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, NULL);

	GLuint tex_vbo;
	glGenBuffers(1, &tex_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, tex_vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(float) * drawing_count, vt, GL_STATIC_DRAW);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_TRUE, 0, NULL);

	delete vp, vn, vt;

	alpha = 0.5;
	beta = 0.5;
	calculateStaticVariables();
}

void PositionBasedBody::set_forces() {
	particle_manager.u_forces.push_back(new gravity(vec3(0, -4, 0)));
	particle_manager.u_forces.push_back(new drag(0.05));
}

void PositionBasedBody::draw() {
	GLfloat *vp = new GLfloat[drawing_count * 3];
	GLfloat	*vn = new GLfloat[drawing_count * 3];
	set_points_normals(vp, vn);

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*drawing_count, vp);
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*drawing_count, vn);

	delete vp, vn;

	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, drawing_count);
}

inline vec3 PositionBasedBody::com() {
	float total_mass = 0;
	vec3 total_mass_displacement = vec3(0, 0, 0);

	for (int i = 0; i < vertex_count; i++) {
		total_mass += vertices[i].m;
		total_mass_displacement += vertices[i].x_extreme*vertices[i].m;
	}

	return total_mass_displacement*(1.0 / total_mass);
}

inline void PositionBasedBody::set_points_normals(float* vp, float* vn) {
	vec3 cross_vertex_0, cross_vertex_1, normal_vec, curr_vertex;
	for (int i = 0; i < drawing_count; i++) {
		curr_vertex = vertices[drawing_order[i]].x;
		if (i % 3 == 0) {
			cross_vertex_0 = vertices[drawing_order[i + 1]].x - curr_vertex;
			cross_vertex_1 = vertices[drawing_order[i + 2]].x - curr_vertex;
		}
		else if (i % 3 == 1){
			cross_vertex_0 = vertices[drawing_order[i + 1]].x - curr_vertex;
			cross_vertex_1 = vertices[drawing_order[i - 1]].x - curr_vertex;
		}
		else {
			cross_vertex_0 = vertices[drawing_order[i - 2]].x - curr_vertex;
			cross_vertex_1 = vertices[drawing_order[i - 1]].x - curr_vertex;
		}
		// Let the GPU handle normalisation
		normal_vec = cross(cross_vertex_0, cross_vertex_1);

		vp[(i * 3)] = curr_vertex.v[0];
		vp[(i * 3) + 1] = curr_vertex.v[1];
		vp[(i * 3) + 2] = curr_vertex.v[2];

		vn[(i * 3)] = normal_vec.v[0];
		vn[(i * 3) + 1] = normal_vec.v[1];
		vn[(i * 3) + 2] = normal_vec.v[2];
	}
}

void PositionBasedBody::calculateDeformity() {
	calculateApq();

	//Calculate Rotation
	// R = Apq*S_inv, S = SQRT(Apq_transpose*Apq)
	mat3 S = sqrt((transpose(Apq))*Apq);

	mat3 test_R = Apq*(inverse(S));

	if (!(test_R == zero_mat3())) {
		R = test_R;
	}

	A = conserveVolume(Apq*Aqq);

	mat3 transformation;

	for (int i = 0; i < 9; i++) {
		transformation.m[i] = R.m[i] * beta + A.m[i] * (1.0 - beta);
	}

	transformation = conserveVolume(transformation);
	
	for (int i = 0; i < vertex_count; i++) {
		vertices[i].x_goal = q[i];
		vertices[i].x_goal = transformation*vertices[i].x_goal;
		vertices[i].x_goal += com();
	}
}

void PositionBasedBody::calculateApq() {
	vec3 c_o_m = com();
	for (int i = 0; i < vertex_count; i++) {
		p[i] = vertices[i].x_extreme - c_o_m;
	}

	mat3 Apq_trans = zero_mat3();
	for (int i = 0; i < vertex_count; i++) {
		float m = vertices[i].m;
		for (int x = 0; x < 3; x++) {
			for (int y = 0; y < 3; y++) {
				Apq_trans.m[x * 3 + y] += m*p[i].v[x] * q[i].v[y];
			}
		}
	}
	Apq = transpose(Apq_trans);
}

void PositionBasedBody::calculateStaticVariables() {
	
	q = new vec3[vertex_count];
	p = new vec3[vertex_count];
	for (int i = 0; i < vertex_count; i++) {
		q[i] = original_shape[i] - original_com;
	}
	
	mat3 Aqq_inv = zero_mat3();
	for (int i = 0; i < vertex_count; i++) {
		float m = vertices[i].m;
		for (int x = 0; x < 3; x++) {
			for (int y = 0; y < 3; y++) {
				Aqq_inv.m[x * 3 + y] += m*q[i].v[x] * q[i].v[y];
			}
		}
	}
	Aqq = inverse(Aqq_inv);
}

void PositionBasedBody::update() {
	particle_manager.update_roof();
	calculateDeformity();
	particle_manager.update(alpha);
}

void PositionBasedBody::non_deformable_update() {
	particle_manager.update_roof();
	particle_manager.non_deformable_update();
}

void PositionBasedBody::stitched_update() {
	particle_manager.update_roof();
	calculateDeformity();
	particle_manager.stick_particles(stitched_points);
	particle_manager.update(alpha);
	particle_manager.stick_particles(stitched_points);
}

void PositionBasedBody::stitched_update(vec3 force) {
	particle_manager.accumulate_forces();
	grab_point->f += force;
	particle_manager.check_planar_collisions();
	particle_manager.apply_forces();
	calculateDeformity();
	particle_manager.stick_particles(stitched_points);
	particle_manager.update(alpha);
	particle_manager.stick_particles(stitched_points);
}
