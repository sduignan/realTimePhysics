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

float increment = 2.5f;
float trans_increment = 0.11f;
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
shader_info tex_shader;

RigidBody physics_cube;
std::vector<RigidBody*> objects;
RigidBodyEngine engine;

int turn = 1;
int choice = 0;
int num_choices = 1;

int text_id;

int num_vertices = 8;
int unique_vertex_indices[] = {0, 1, 2, 3, 4, 6, 7, 10};

bool shifting = false;

bool pause = false;

mat4 plane_orientation_mat = identity_mat4();
vec3 plane_normal = vec3(0, 1, 0);
float plane_dist = 1.0;

int current_object = 0;

//keyboard control
void My_Key_Callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) {
			case GLFW_KEY_ESCAPE:
					close_window = true;
					break;
			case GLFW_KEY_LEFT_SHIFT:
			case GLFW_KEY_RIGHT_SHIFT:
				shifting = !shifting;
				break;
			case GLFW_KEY_LEFT: {
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*minus_z_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*z_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*minus_x_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*x_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*minus_y_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->R = objects[current_object]->R*y_rot;
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
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[0] -= trans_increment;
				}
				else {
					move.v[0] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_D: {
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[0] += trans_increment;
				}
				else {
					move.v[0] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_W: {
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[2] -= trans_increment;
				}
				else {
					move.v[2] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_S: {
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[2] += trans_increment;
				}
				else {
					move.v[2] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_Q: {
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[1] -= trans_increment;
				}
				else {
					move.v[1] -= trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_E: {
				if (choice == 0 && !shifting) {
					objects[current_object]->x.v[1] += trans_increment;
				}
				else {
					move.v[1] += trans_increment;
					cam_moved = true;
				}
				break;
			}
			case GLFW_KEY_U: {
				plane_normal = normalise(minus_z_rot*plane_normal);
				plane_orientation_mat = mat3_to_mat4(minus_z_rot)*plane_orientation_mat;
				break;
			}
			case GLFW_KEY_O: {
				plane_normal = normalise(z_rot*plane_normal);
				plane_orientation_mat = mat3_to_mat4(z_rot)*plane_orientation_mat;
				break;
			}
			case GLFW_KEY_SPACE: {
				current_object = (current_object + 1) % 2;
				break;
			}
			case GLFW_KEY_ENTER:{
				objects[current_object]->reset();
				plane_normal = vec3(0, 1, 0);
				plane_orientation_mat = identity_mat4();
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
	
	//Set line width
	glLineWidth(2.0f);

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
	scene_object physics_cube_model("Meshes/solid_cube.obj", "Textures/cube.png");
	scene_object plane("Meshes/plane.obj", "Textures/ForestFloor.png");

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
	GLuint c_loc = glGetUniformLocation(spring_shader.shader_id, "colour");

	//Textured shader
	tex_shader.shader_id = create_programme_from_files("Shaders/tex.vert", "Shaders/tex.frag");
	glUseProgram(spring_shader.shader_id);
	//get locations of M, V, P matrices
	tex_shader.M_loc = glGetUniformLocation(tex_shader.shader_id, "M");
	assert(tex_shader.M_loc > -1);
	tex_shader.V_loc = glGetUniformLocation(tex_shader.shader_id, "V");
	assert(tex_shader.V_loc > -1);
	tex_shader.P_loc = glGetUniformLocation(tex_shader.shader_id, "P");
	assert(tex_shader.P_loc > -1);

	objects.push_back(new RigidBody);
	objects[0]->set_mesh("Meshes/solid_cube.obj");
	objects[0]->spring.spring_origin = vec3(0, -3, 0);
	objects[0]->set_spring_vao(c_loc);

	physics_cube.set_mesh("Meshes/solid_cube.obj");
	physics_cube.spring.spring_origin = vec3(0, -3, 0);
	physics_cube.set_spring_vao(c_loc);

	objects.push_back(new RigidBody);
	objects[1]->set_mesh("Meshes/octahedron.obj");
	objects[1]->octahedron_setup(10, 1.414214, 2, identity_mat3());
	objects[1]->spring.spring_origin = vec3(0, -3, 0);
	objects[1]->set_spring_vao(c_loc);


	GLuint cube_tex_handle;
	load_texture("Textures/cube_tex.png", &cube_tex_handle);
	GLuint octa_tex_handle;
	load_texture("Textures/octahedron.png", &octa_tex_handle);


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


	glUseProgram(tex_shader.shader_id);
	glUniformMatrix4fv(tex_shader.P_loc, 1, GL_FALSE, proj_mat.m);
	glUniformMatrix4fv(tex_shader.V_loc, 1, GL_FALSE, view_mat.m);


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

	mat4 sphere_model_mat = identity_mat4();
	mat4 cube_model_mat = identity_mat4();
	vec3 colour;
	mat4 plane_model_mat = translate(identity_mat4(), vec3(0, -1, 0));

	objects[0]->x = vec3(0, 1, 0);
	objects[0]->centre = vec3(0, 1, 0);
	objects[1]->x = vec3(0, 1, 0);
	objects[1]->centre = vec3(0, 1, 0);


	//MAIN LOOP
	while (!glfwWindowShouldClose(g_window)) {
		// wipe the drawing surface clear
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render a sky-box using the cube-map texture
		sky_cube.draw();
	
		//plane
		glUseProgram(tex_shader.shader_id);
		glUniformMatrix4fv(tex_shader.V_loc, 1, GL_FALSE, view_mat.m);
		plane_model_mat = translate(identity_mat4(), plane_normal*-plane_dist);
		glUniformMatrix4fv(tex_shader.M_loc, 1, GL_FALSE, (plane_orientation_mat*plane_model_mat).m);
		plane.draw();


		glUniformMatrix4fv(tex_shader.M_loc, 1, GL_FALSE, identity_mat4().m);
		//rigid body
		//glUseProgram(simple_shader.shader_id);
		//glUniformMatrix4fv(simple_shader.V_loc, 1, GL_FALSE, view_mat.m);


		glActiveTexture(GL_TEXTURE0);
		if (current_object == 0) {
			glBindTexture(GL_TEXTURE_2D, cube_tex_handle);
		}
		else if (current_object == 1) {
			glBindTexture(GL_TEXTURE_2D, octa_tex_handle);
		}
		objects[current_object]->draw();

		if (shifting) {
			engine.update_body(*objects[current_object], plane_normal, plane_dist);
		}

		objects[current_object]->update_vertices();


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