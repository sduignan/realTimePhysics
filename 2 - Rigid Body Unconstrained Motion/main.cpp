#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <string>
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

//3rd party code
#include "Antons_maths_funcs.h"
#include "obj_parser.h"
#include "text.h"
#include "gl_utils.h" // common opengl functions and small utilities like logs

#define STBI_ASSERT(x)

//My own classes
#include "scene_object.h"
#include "cube_map.h"
#include "rigidBody.h"
#include "rigidBodyEngine.h"

#define FRONT "cube_map/negz.jpg"
#define BACK "cube_map/posz.jpg"
#define TOP "cube_map/posy.jpg"
#define BOTTOM "cube_map/negy.jpg"
#define LEFT "cube_map/negx.jpg"
#define RIGHT "cube_map/posx.jpg"

int g_gl_width = 1200;
int g_gl_height = 800;
bool close_window = false;
GLFWwindow* g_window = NULL;

float model_rotation = 0.0f;

// camera matrices. it's easier if they are global
mat4 view_mat;
mat4 proj_mat;
vec3 cam_pos(0.0f, 0.0f, 5.0f);

bool cam_moved = false;
vec3 move(0.0, 0.0, 0.0);
float cam_yaw = 0.0f; // y-rotation in degrees
float cam_pitch = 0.0f;
float cam_roll = 0.0;

float increment = 0.5f;
float trans_increment = 0.05f;
float cam_heading = 0.0f; // y-rotation in degrees

float cos_theta = 0.99619469809;
float sin_theta = 0.08715574274;

mat3 x_rot = mat3(1, 0, 0, 0, cos_theta, -sin_theta, 0, sin_theta, cos_theta);
mat3 minus_x_rot = mat3(1, 0, 0, 0, cos_theta, sin_theta, 0, -sin_theta, cos_theta);
mat3 y_rot = mat3(cos_theta, 0, sin_theta, 0, 1, 0, -sin_theta, 0, cos_theta);
mat3 minus_y_rot = mat3(cos_theta, 0, -sin_theta, 0, 1, 0, sin_theta, 0, cos_theta);
mat3 z_rot = mat3(cos_theta, -sin_theta, 0, sin_theta, cos_theta, 0, 0, 0, 1);
mat3 minus_z_rot = mat3(cos_theta, sin_theta, 0, -sin_theta, cos_theta, 0, 0, 0, 1);

// keep track of some useful vectors that can be used for keyboard movement
vec4 fwd(0.0f, 0.0f, -1.0f, 0.0f);
vec4 rgt(1.0f, 0.0f, 0.0f, 0.0f);
vec4 up(0.0f, 1.0f, 0.0f, 0.0f);

versor q;

struct shader_info {
	GLuint shader_id;
	int M_loc;
	int V_loc;
	int P_loc;
};
shader_info simple_shader;
shader_info spring_shader;

RigidBody physics_cube;
RigidBodyEngine engine;

int turn = 1;
int choice = 0;
int num_choices = 4;

int text_id;

int num_vertices = 8;
int unique_vertex_indices[] = {0, 1, 2, 3, 4, 6, 7, 10};

//keyboard control
void My_Key_Callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) {
			case GLFW_KEY_ESCAPE:
					close_window = true;
					break;
			case GLFW_KEY_LEFT: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*minus_z_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[2] -= increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[2] -= increment;
				}
				else {
					cam_yaw += increment;
					cam_moved = true;
					versor q_yaw = quat_from_axis_deg(cam_yaw, up.v[0], up.v[1], up.v[2]);
					q = q_yaw * q;
				}
				break;
			}
			case GLFW_KEY_RIGHT: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*z_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[2] += increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[2] += increment;
				}
				else {
					cam_yaw -= increment;
					cam_moved = true;
					versor q_yaw = quat_from_axis_deg(
						cam_yaw, up.v[0], up.v[1], up.v[2]
					);
					q = q_yaw * q;
				}
				break;
			}
			case GLFW_KEY_UP: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*minus_x_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[0] -= increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[0] -= increment;
				}
				else {
					cam_pitch += increment;
					cam_moved = true;
					versor q_pitch = quat_from_axis_deg(
						cam_pitch, rgt.v[0], rgt.v[1], rgt.v[2]
					);
					q = q_pitch * q;
				}
				break;
			}
			case GLFW_KEY_DOWN: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*x_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[0] += increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[0] += increment;
				}
				else {
					cam_pitch -= increment;
					cam_moved = true;
					versor q_pitch = quat_from_axis_deg(
						cam_pitch, rgt.v[0], rgt.v[1], rgt.v[2]
					);
					q = q_pitch * q;
				}
				break;
			}
			case GLFW_KEY_Z: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*minus_y_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[1] -= increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[1] -= increment;
				}
				else {
					cam_roll -= increment;
					cam_moved = true;
					versor q_roll = quat_from_axis_deg(
						cam_roll, fwd.v[0], fwd.v[1], fwd.v[2]
					);
					q = q_roll * q;
				}
				break;
			}
			case GLFW_KEY_C: {
				if (choice == 0) {
					physics_cube.R = physics_cube.R*y_rot;
				}
				else if (choice == 1) {
					physics_cube.L.v[1] += increment;
				}
				else if (choice == 2) {
					physics_cube.torque.v[1] += increment;
				}
				else {
					cam_roll -= increment;
					cam_moved = true;
					versor q_roll = quat_from_axis_deg(
						cam_roll, fwd.v[0], fwd.v[1], fwd.v[2]
					);
					q = q_roll * q;
				}
				break;
			}
			case GLFW_KEY_A: {
				if (choice == 0) {
					physics_cube.x.v[0] -= trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[0] -= increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[0] -= increment;
				}
				else {
					move.v[0] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_D: {
				if (choice == 0) {
					physics_cube.x.v[0] += trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[0] += increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[0] += increment;
				}
				else {
					move.v[0] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_W: {
				if (choice == 0) {
					physics_cube.x.v[2] -= trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[2] -= increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[2] -= increment;
				}
				else {
					move.v[2] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_S: {
				if (choice == 0) {
					physics_cube.x.v[2] += trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[2] += increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[2] += increment;
				}
				else {
					move.v[2] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_Q: {
				if (choice == 0) {
					physics_cube.x.v[1] -= trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[1] -= increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[1] -= increment;
				}
				else {
					move.v[1] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_E: {
				if (choice == 0) {
					physics_cube.x.v[1] += trans_increment;
				}
				else if (choice == 1) {
					physics_cube.P.v[1] += increment;
				}
				else if (choice == 2) {
					physics_cube.force.v[1] += increment;
				}
				else {
					move.v[1] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_SPACE: {
				choice = (choice + 1) % num_choices;
				if (choice == 1) {
					physics_cube.force = vec3(0.0, 0.0, 0.0);
					physics_cube.torque = vec3(0.0, 0.0, 0.0);
				}
				break;
			}
			case GLFW_KEY_ENTER: {
				physics_cube.reset();
				break;
			}
		}
	}
}

int main() {
	//START OPENGL
	restart_gl_log();
	// start GL context and O/S window using the GLFW helper library
	start_gl();

	// Tell the window where to find its key callback function
	glfwSetKeyCallback(g_window, My_Key_Callback);
	
	//TEXT setup
	if (!init_text_rendering("freemono.png", "freemono.meta", g_gl_width, g_gl_height))
	{
		fprintf(stderr, "ERROR init text rendering\n");
		exit(1);
	}
	
	text_id = add_text("", -0.9, 0.9, 15, 0.0, 0.0, 0.0, 1.0);

	// Set up vertex buffers and vertex array objects
	scene_object ball("Meshes/ball.obj", "Textures/angel_diffuse.png", "Textures/angel_normals.png");
	scene_object cube("Meshes/cube.obj", "Textures/angel_diffuse.png");
	scene_object physics_cube_model("Meshes/solid_cube.obj", "Textures/angel_diffuse.png");

	physics_cube.set_mesh("Meshes/solid_cube.obj");
	physics_cube.set_spring_vao();

	
	//Load Simpler Pear
	{
		FILE* fp = fopen("Meshes/simplerPear.obj", "r");
		if (!fp) {
			fprintf(stderr, "ERROR: could not find file %s\n", "Meshes/simplerPear.obj");
			return false;
		}

		int pear_point_count = 0;
		bool done = false;
		char line[1024];
		while (fgets(line, 1024, fp) && !done) {
			if (line[0] == 'v') {
				if (line[1] == ' ') {
					pear_point_count++;
				}
				else {
					done = true;
				}
			}
		}
		
		int current = 0;
		done = false;

		rewind(fp);
		while (fgets(line, 1024, fp) && !done) {
			// vertex
			if (line[0] == 'v') {
				// vertex point
				if (line[1] == ' ') {
					float x, y, z;
					x = y = z = 0.0f;
					sscanf(line, "v %f %f %f", &x, &y, &z);
					current++;
				}
			}
			if (current == pear_point_count){
				done = true;
			}
		}
		fclose(fp);


	}


	//CUBE MAP
	cube_map sky_cube(FRONT, BACK, TOP, BOTTOM, LEFT, RIGHT);

	// Ball Shader
	simple_shader.shader_id = create_programme_from_files("Shaders/light.vert", "Shaders/light.frag");
	glUseProgram(simple_shader.shader_id);
	//get locations of M, V, P matrices
	simple_shader.M_loc = glGetUniformLocation(simple_shader.shader_id, "M");
	assert(simple_shader.M_loc > -1);
	simple_shader.V_loc = glGetUniformLocation(simple_shader.shader_id, "V");
	assert(simple_shader.V_loc > -1);
	simple_shader.P_loc = glGetUniformLocation(simple_shader.shader_id, "P");
	assert(simple_shader.P_loc > -1);

	// Cube Shader
	spring_shader.shader_id = create_programme_from_files("Shaders/string.vert", "Shaders/string.frag");
	glUseProgram(spring_shader.shader_id);
	//get locations of M, V, P matrices
	spring_shader.M_loc = glGetUniformLocation(spring_shader.shader_id, "M");
	assert(spring_shader.M_loc > -1);
	spring_shader.V_loc = glGetUniformLocation(spring_shader.shader_id, "V");
	assert(spring_shader.V_loc > -1);
	spring_shader.P_loc = glGetUniformLocation(spring_shader.shader_id, "P");
	assert(spring_shader.P_loc > -1);



	// cube-map shaders
	sky_cube.shader = create_programme_from_files("Shaders/sky_cube.vert", "Shaders/sky_cube.frag");
	glUseProgram(sky_cube.shader);
	// note that this view matrix should NOT contain camera translation.
	sky_cube.proj_loc = glGetUniformLocation(sky_cube.shader, "P");
	sky_cube.view_loc = glGetUniformLocation(sky_cube.shader, "V");


	//CREATE CAMERA
	// input variables
	float near = 0.1f; // clipping plane
	float far = 100.0f; // clipping plane
	float fovy = 67.0f; // 67 degrees
	float aspect = (float)g_gl_width / (float)g_gl_height; // aspect ratio
	proj_mat = perspective(fovy, aspect, near, far);

	mat4 T = translate(
		identity_mat4(), vec3(-cam_pos.v[0], -cam_pos.v[1], -cam_pos.v[2])
	);
	mat4 R = rotate_y_deg(identity_mat4(), -cam_heading);
	q = quat_from_axis_deg(-cam_heading, 0.0f, 1.0f, 0.0f);
	view_mat = R * T;


	//SET RENDERING DEFAULTS
	glUseProgram(simple_shader.shader_id);
	glUniformMatrix4fv(simple_shader.P_loc, 1, GL_FALSE, proj_mat.m);
	glUniformMatrix4fv(simple_shader.V_loc, 1, GL_FALSE, view_mat.m);
	glUniformMatrix4fv(simple_shader.M_loc, 1, GL_FALSE, identity_mat4().m);


	glUseProgram(spring_shader.shader_id);
	glUniformMatrix4fv(spring_shader.P_loc, 1, GL_FALSE, proj_mat.m);
	glUniformMatrix4fv(spring_shader.V_loc, 1, GL_FALSE, view_mat.m);
	glUniformMatrix4fv(spring_shader.M_loc, 1, GL_FALSE, identity_mat4().m);

	sky_cube.proj = proj_mat;
	sky_cube.view = R;
	sky_cube.update_mats(true, true);

	mat4 ball_model_mat = scale(identity_mat4(), vec3(1.5, 1.5, 1.5));

	glEnable(GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"
	glEnable(GL_CULL_FACE); // cull face
	glCullFace(GL_BACK); // cull back face
	glFrontFace(GL_CCW); // set counter-clock-wise vertex order to mean the front
	glClearColor(0.2, 0.2, 0.2, 1.0); // grey background to help spot mistakes
	glViewport(0, 0, g_gl_width, g_gl_height);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	mat4 cube_model_mat = identity_mat4();

	//MAIN LOOP
	while (!glfwWindowShouldClose(g_window)) {
		// wipe the drawing surface clear
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render a sky-box using the cube-map texture
		sky_cube.draw();
	
		//rigid body
		glUseProgram(simple_shader.shader_id);
		glUniformMatrix4fv(simple_shader.V_loc, 1, GL_FALSE, view_mat.m);
		physics_cube.draw();
		
		if (choice == 3) {
			glUseProgram(spring_shader.shader_id);
			glUniformMatrix4fv(spring_shader.V_loc, 1, GL_FALSE, view_mat.m);
			physics_cube.draw_spring();
			engine.update_body(physics_cube);
		}
		else if (choice >= 1) {
			engine.apply_forces_and_torque(physics_cube);
		}


		physics_cube.update_vertices();


		std::string description = "";
		description += "Displacement:      (" + std::to_string(physics_cube.x.v[0]) + ", " + std::to_string(physics_cube.x.v[1]) + ", " + std::to_string(physics_cube.x.v[2]) + ")\n";
		description += "Linear Momentum:  (" + std::to_string(physics_cube.P.v[0]) + ", " + std::to_string(physics_cube.P.v[1]) + ", " + std::to_string(physics_cube.P.v[2]) + ")\n";
		description += "Total Force:       (" + std::to_string(physics_cube.force.v[0]) + ", " + std::to_string(physics_cube.force.v[1]) + ", " + std::to_string(physics_cube.force.v[2]) + ")\n\n";
		description += "Orientation:       |" + std::to_string(physics_cube.R.m[0]) + ", " + std::to_string(physics_cube.R.m[3]) + ", " + std::to_string(physics_cube.R.m[6]) + "|\n";
		description += "                   |" + std::to_string(physics_cube.R.m[1]) + ", " + std::to_string(physics_cube.R.m[4]) + ", " + std::to_string(physics_cube.R.m[7]) + "|\n";
		description += "                   |" + std::to_string(physics_cube.R.m[2]) + ", " + std::to_string(physics_cube.R.m[5]) + ", " + std::to_string(physics_cube.R.m[8]) + "|\n\n";
		description += "Angular Momentum: (" + std::to_string(physics_cube.L.v[0]) + ", " + std::to_string(physics_cube.L.v[1]) + ", " + std::to_string(physics_cube.L.v[2]) + ")\n";
		description += "Total Torque:      (" + std::to_string(physics_cube.torque.v[0]) + ", " + std::to_string(physics_cube.torque.v[1]) + ", " + std::to_string(physics_cube.torque.v[2]) + ")\n";
		
		description += "\n\n\nVertices:\n";

		for (int j = 0; j < num_vertices; j++) {
			int i = unique_vertex_indices[j];
			description += "        (" + std::to_string(physics_cube.current_vertices[i].v[0]) + ", " + std::to_string(physics_cube.current_vertices[i].v[1]) + ", " + std::to_string(physics_cube.current_vertices[i].v[2]) + ")\n";
		}

		update_text(text_id, description.c_str());

		//Write scene description on frame
		draw_texts();

		// update view matrix
		if (cam_moved) {
			cam_heading += cam_yaw;

			// re-calculate local axes so can move fwd in dir cam is pointing
			R = quat_to_mat4(q);
			fwd = R * vec4(0.0, 0.0, -1.0, 0.0);
			rgt = R * vec4(1.0, 0.0, 0.0, 0.0);
			up = R * vec4(0.0, 1.0, 0.0, 0.0);

			cam_pos = cam_pos + vec3(fwd) * -move.v[2];
			cam_pos = cam_pos + vec3(up) * move.v[1];
			cam_pos = cam_pos + vec3(rgt) * move.v[0];
			mat4 T = translate(identity_mat4(), vec3(cam_pos));

			view_mat = inverse(R) * inverse(T);

			// cube-map view matrix has rotation, but not translation
			sky_cube.view = inverse(R);
			sky_cube.update_mats(false, true);
		}

		cam_moved = false;
		cam_yaw = 0.0f;
		cam_pitch = 0.0f;
		cam_roll = 0.0;
		move = vec3(0.0, 0.0, 0.0);

		glfwPollEvents();
		glfwSwapBuffers(g_window);
		if (close_window) {
			glfwDestroyWindow(g_window);
		}
	}
}