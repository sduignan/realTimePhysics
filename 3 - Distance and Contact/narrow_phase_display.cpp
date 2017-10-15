#include "narrow_phase_display.h"

NarrowPhaseDisplay::NarrowPhaseDisplay() {
	point = vec3(-1, -2, 1);

	triangle[0] = vec3(0.0, 0.0, 0.0);
	triangle[1] = vec3(0.0, 1.0, 0.0);
	triangle[2] = vec3(-1.0, 0.0, 0.0);

	triangle_normals[0] = vec3(0.0, 0.0, 1.0);
	triangle_normals[1] = vec3(0.0, 0.0, 1.0);
	triangle_normals[2] = vec3(0.0, 0.0, 1.0);

	triangle_b[0] = vec3(1.0, 1.0, 0.2);
	triangle_b[1] = vec3(1.3, 2.4, 0.25);
	triangle_b[2] = vec3(0.33, 2.2, 0.1);

	triangle_b_normals[0] = normalise(cross(triangle_b[1] - triangle_b[0], triangle_b[2] - triangle_b[0]));
	triangle_b_normals[1] = normalise(cross(triangle_b[2] - triangle_b[1], triangle_b[0] - triangle_b[1]));
	triangle_b_normals[2] = normalise(cross(triangle_b[0] - triangle_b[2], triangle_b[1] - triangle_b[2]));
}

void NarrowPhaseDisplay::setup(GLuint tri_shader, GLuint line_shader, int tri_col_loc, int line_col_loc) {
	triangle_shader_id = tri_shader;
	line_shader_id = line_shader;

	triangle_colour_loc = tri_col_loc;
	line_colour_loc = line_col_loc;

	float vp[] = {triangle[0].v[0], triangle[0].v[1], triangle[0].v[2], triangle[1].v[0], triangle[1].v[1], triangle[1].v[2], triangle[2].v[0], triangle[2].v[1], triangle[2].v[2]};
	
	glGenVertexArrays(1, &triangle_vao);
	glBindVertexArray(triangle_vao);
	glGenBuffers(1, &triangle_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * 3, vp, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	float vn[] = { triangle_normals[0].v[0], triangle_normals[0].v[1], triangle_normals[0].v[2], triangle_normals[1].v[0], triangle_normals[1].v[1], triangle_normals[1].v[2], triangle_normals[2].v[0], triangle_normals[2].v[1], triangle_normals[2].v[2] };

	glGenBuffers(1, &normals_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float)*3, vn, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, NULL);

	float b_vp[] = { triangle_b[0].v[0], triangle_b[0].v[1], triangle_b[0].v[2], triangle_b[1].v[0], triangle_b[1].v[1], triangle_b[1].v[2], triangle_b[2].v[0], triangle_b[2].v[1], triangle_b[2].v[2] };

	glGenVertexArrays(1, &triangle_b_vao);
	glBindVertexArray(triangle_b_vao);
	glGenBuffers(1, &triangle_b_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_b_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * 3, b_vp, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	float b_vn[] = { triangle_b_normals[0].v[0], triangle_b_normals[0].v[1], triangle_b_normals[0].v[2], triangle_b_normals[1].v[0], triangle_b_normals[1].v[1], triangle_b_normals[1].v[2], triangle_b_normals[2].v[0], triangle_b_normals[2].v[1], triangle_b_normals[2].v[2] };

	glGenBuffers(1, &triangle_b_normals_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_b_normals_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * 3, b_vn, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, NULL);


	float vp_line[] = { point.v[0], point.v[1], point.v[2], point.v[0], point.v[1], point.v[2]};

	glGenVertexArrays(1, &line_vao);
	glBindVertexArray(line_vao);
	glGenBuffers(1, &line_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * 2, vp_line, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
}

float NarrowPhaseDisplay::update_and_draw() {
	int region;
	vec3 end = engine.point_to_triangle(point, triangle[0], triangle[1], triangle[2], region);
	
	//Update line vao:
	float vp[] = { point.v[0], point.v[1], point.v[2], end.v[0], end.v[1], end.v[2] };
	glBindVertexArray(line_vao);
	glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);

	glUseProgram(line_shader_id);
	glUniform3fv(line_colour_loc, 1, vec3(0.0, 0.0, 0.0).v);
	glBindVertexArray(line_vao);
	glDrawArrays(GL_LINES, 0, 2);
	glUniform3fv(line_colour_loc, 1, vec3(0.05, 0.9, 0.25).v);
	glDrawArrays(GL_POINTS, 0, 2);
	
	//Lowlight attached region
	glUniform3fv(line_colour_loc, 1, vec3(0.8, 0.1, 0.16).v);
	if (region < 3) {
		glBindVertexArray(triangle_vao);
		glPointSize(6);
		glDrawArrays(GL_POINTS, region, 1);
		glPointSize(4);
	}
	else if (region < 6) {
		int index = region - 3;
		for (int i = 0; i < 3; i++) {
			vp[i] = triangle[index].v[i];
			vp[i + 3] = triangle[(index + 1) % 3].v[i];
		}
		glBindVertexArray(line_vao);
		glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);
		glDrawArrays(GL_LINES, 0, 2);
	}
	else {
		glBindVertexArray(triangle_vao);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}

	//update triangle vao:
	float tri_vp[] = { triangle[0].v[0], triangle[0].v[1], triangle[0].v[2], triangle[1].v[0], triangle[1].v[1], triangle[1].v[2], triangle[2].v[0], triangle[2].v[1], triangle[2].v[2] };
	float vn[] = { triangle_normals[0].v[0], triangle_normals[0].v[1], triangle_normals[0].v[2], triangle_normals[1].v[0], triangle_normals[1].v[1], triangle_normals[1].v[2], triangle_normals[2].v[0], triangle_normals[2].v[1], triangle_normals[2].v[2] };
	
	glBindVertexArray(triangle_vao);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*3, tri_vp);

	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float)*3, vn);

	glUseProgram(triangle_shader_id);
	glUniform3fv(triangle_colour_loc, 1, vec3(1.0, 0.65, 0.95).v);
	glBindVertexArray(triangle_vao);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	
	return length(point - end);
}

float NarrowPhaseDisplay::find_edge_point_and_draw() {
	vec3 end;
	float dist = engine.point_to_line_dist(point, triangle[0], triangle[1], end);

	//Update line vao:
	float vp[] = { point.v[0], point.v[1], point.v[2], end.v[0], end.v[1], end.v[2] };
	glBindVertexArray(line_vao);
	glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);

	glUseProgram(line_shader_id);
	glUniform3fv(line_colour_loc, 1, vec3(0.0, 0.0, 0.0).v);
	glBindVertexArray(line_vao);
	glDrawArrays(GL_LINES, 0, 2);
	glUniform3fv(line_colour_loc, 1, vec3(0.05, 0.9, 0.25).v);
	glDrawArrays(GL_POINTS, 0, 2);
	//draw attached line
	glUniform3fv(line_colour_loc, 1, vec3(0.8, 0.1, 0.16).v);
	glBindVertexArray(triangle_vao);
	glDrawArrays(GL_LINES, 0, 2);

	//update triangle vao:
	float tri_vp[] = { triangle[0].v[0], triangle[0].v[1], triangle[0].v[2], triangle[1].v[0], triangle[1].v[1], triangle[1].v[2], triangle[2].v[0], triangle[2].v[1], triangle[2].v[2] };
	float vn[] = { triangle_normals[0].v[0], triangle_normals[0].v[1], triangle_normals[0].v[2], triangle_normals[1].v[0], triangle_normals[1].v[1], triangle_normals[1].v[2], triangle_normals[2].v[0], triangle_normals[2].v[1], triangle_normals[2].v[2] };

	glBindVertexArray(triangle_vao);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 3, tri_vp);

	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 3, vn);

	glUseProgram(triangle_shader_id);
	glUniform3fv(triangle_colour_loc, 1, vec3(1.0, 0.65, 0.95).v);
	glBindVertexArray(triangle_vao);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	return dist;
}

float NarrowPhaseDisplay::t_to_t_update_and_draw() {
	vec3 pa, pb;
	int region[2];

	float dist = engine.triangle_to_triangle(triangle, triangle_b, pa, pb, region);

	//Update line vao:
	float vp[] = { pa.v[0], pa.v[1], pa.v[2], pb.v[0], pb.v[1], pb.v[2] };
	glBindVertexArray(line_vao);
	glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);

	glUseProgram(line_shader_id);
	glUniform3fv(line_colour_loc, 1, vec3(0.0, 0.0, 0.0).v);
	glBindVertexArray(line_vao);
	glDrawArrays(GL_LINES, 0, 2);
	glUniform3fv(line_colour_loc, 1, vec3(0.05, 0.9, 0.25).v);
	glDrawArrays(GL_POINTS, 0, 2);
	
	//update triangle_b vao:
	float tri_vp[] = { triangle_b[0].v[0], triangle_b[0].v[1], triangle_b[0].v[2], triangle_b[1].v[0], triangle_b[1].v[1], triangle_b[1].v[2], triangle_b[2].v[0], triangle_b[2].v[1], triangle_b[2].v[2] };
	float vn[] = { triangle_b_normals[0].v[0], triangle_b_normals[0].v[1], triangle_b_normals[0].v[2], triangle_b_normals[1].v[0], triangle_b_normals[1].v[1], triangle_b_normals[1].v[2], triangle_b_normals[2].v[0], triangle_b_normals[2].v[1], triangle_b_normals[2].v[2] };

	glBindVertexArray(triangle_b_vao);
	glBindBuffer(GL_ARRAY_BUFFER, triangle_b_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 3, tri_vp);

	glBindBuffer(GL_ARRAY_BUFFER, triangle_b_normals_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 3, vn);


	//Lowlight attached region
	glUniform3fv(line_colour_loc, 1, vec3(0.8, 0.1, 0.16).v);
	if (region[0] < 6) {
		vec3 *b;
		GLuint *vao_a, *vao_b;
		if (region[0] < 3) {
			b = triangle_b;
			vao_a = &triangle_vao;
			vao_b = &triangle_b_vao;
		}
		else {
			b = triangle;
			vao_a = &triangle_b_vao;
			vao_b = &triangle_vao;
		}
		glBindVertexArray(*vao_a);
		glPointSize(6);
		glDrawArrays(GL_POINTS, region[0]%3, 1);
		glPointSize(4);
		if (region[1] < 3) {
			glBindVertexArray(*vao_b);
			glPointSize(6);
			glDrawArrays(GL_POINTS, region[1], 1);
			glPointSize(4);
		}
		else if (region[1] < 6) {
			int index = region[1] - 3;
			for (int i = 0; i < 3; i++) {
				vp[i] = b[index].v[i];
				vp[i + 3] = b[(index + 1) % 3].v[i];
			}
			glBindVertexArray(line_vao);
			glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
			glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);
			glDrawArrays(GL_LINES, 0, 2);
		}
		else {
			glBindVertexArray(*vao_b);
			glDrawArrays(GL_TRIANGLES, 0, 3);
		}
	}
	else {
		//Line on first triangle
		int index = (region[0] - 6)/3;
		for (int i = 0; i < 3; i++) {
			vp[i] = triangle[index].v[i];
			vp[i + 3] = triangle[(index + 1) % 3].v[i];
		}
		glBindVertexArray(line_vao);
		glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);
		glDrawArrays(GL_LINES, 0, 2);
		//Line on second triangle
		index = region[1] % 3;
		for (int i = 0; i < 3; i++) {
			vp[i] = triangle_b[index].v[i];
			vp[i + 3] = triangle_b[(index + 1) % 3].v[i];
		}
		glBindVertexArray(line_vao);
		glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(float) * 2, vp);
		glDrawArrays(GL_LINES, 0, 2);
	}

	glUseProgram(triangle_shader_id);
	glUniform3fv(triangle_colour_loc, 1, vec3(1.0, 0.65, 0.95).v);
	glBindVertexArray(triangle_vao);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	glUniform3fv(triangle_colour_loc, 1, vec3(0.15, 0.2, 1.00).v);
	glBindVertexArray(triangle_b_vao);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	return dist;
}