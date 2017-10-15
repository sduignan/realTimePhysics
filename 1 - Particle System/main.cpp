#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

//3rd party code
#include "Antons_maths_funcs.h"
#include "obj_parser.h"
#include "gl_utils.h" // common opengl functions and small utilities like logs

#define STBI_ASSERT(x)

//My own classes
#include "scene_object.h"
#include "cube_map.h"
#include "particle.h"
#include "unaryForces.h"
#include "particleSystem.h"

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

// keep track of some useful vectors that can be used for keyboard movement
vec4 fwd(0.0f, 0.0f, -1.0f, 0.0f);
vec4 rgt(1.0f, 0.0f, 0.0f, 0.0f);
vec4 up(0.0f, 1.0f, 0.0f, 0.0f);

versor q;

std::vector<unaryForce*> forces;
std::vector<pointForce*> pear;
particleSystem p;

vec3 *pear_points;

mat4 locations[NUM_PARTICLES];

struct shader_info {
	GLuint shader_id;
	int M_loc;
	int V_loc;
	int P_loc;
};
shader_info simple_shader;
shader_info cube_shader;

int turn = 1;
int choice = 0;

//keyboard control
void My_Key_Callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) {
			case GLFW_KEY_ESCAPE:
					close_window = true;
					break;
			case GLFW_KEY_LEFT: {
				cam_yaw += increment;
				cam_moved = true;
				versor q_yaw = quat_from_axis_deg(cam_yaw, up.v[0], up.v[1], up.v[2]);
				q = q_yaw * q;
				break;
			}
			case GLFW_KEY_RIGHT: {
				cam_yaw -= increment;
				cam_moved = true;
				versor q_yaw = quat_from_axis_deg(
					cam_yaw, up.v[0], up.v[1], up.v[2]
				);
				q = q_yaw * q;
				break;
			}
			case GLFW_KEY_UP: {
				cam_pitch += increment;
				cam_moved = true;
				versor q_pitch = quat_from_axis_deg(
					cam_pitch, rgt.v[0], rgt.v[1], rgt.v[2]
				);
				q = q_pitch * q;
				break;
			}
			case GLFW_KEY_DOWN: {
				cam_pitch -= increment;
				cam_moved = true;
				versor q_pitch = quat_from_axis_deg(
					cam_pitch, rgt.v[0], rgt.v[1], rgt.v[2]
				);
				q = q_pitch * q;
				break;
			}
			case GLFW_KEY_Z: {
				cam_roll -= increment;
				cam_moved = true;
				versor q_roll = quat_from_axis_deg(
					cam_roll, fwd.v[0], fwd.v[1], fwd.v[2]
				);
				q = q_roll * q;
				break;
			}
			case GLFW_KEY_C: {
				cam_roll -= increment;
				cam_moved = true;
				versor q_roll = quat_from_axis_deg(
					cam_roll, fwd.v[0], fwd.v[1], fwd.v[2]
				);
				q = q_roll * q;
				break;
			}
			case GLFW_KEY_A: {
				move.v[0] -= trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_D: {
				move.v[0] += trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_W: {
				move.v[2] -= trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_S: {
				move.v[2] += trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_Q: {
				move.v[1] -= trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_E: {
				move.v[1] += trans_increment;
				cam_moved = true;
				break;
			}
			case GLFW_KEY_R: {
				p.toggle_rotation();
				break;
			}
			case GLFW_KEY_SPACE: {
				p.effect = !p.effect;
				break;
			}
			case GLFW_KEY_ENTER: {
				p.reset();
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

	// Set up vertex buffers and vertex array objects
	scene_object ball("Meshes/ball.obj", "Textures/angel_diffuse.png", "Textures/angel_normals.png");
	scene_object cube("Meshes/cube.obj", "Textures/angel_diffuse.png");

	
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

		pear_points = new vec3[pear_point_count];

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
					pear_points[current] = vec3(x, y, z);
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
	cube_shader.shader_id = create_programme_from_files("Shaders/alpha.vert", "Shaders/alpha.frag");
	glUseProgram(cube_shader.shader_id);
	//get locations of M, V, P matrices
	cube_shader.M_loc = glGetUniformLocation(cube_shader.shader_id, "M");
	assert(cube_shader.M_loc > -1);
	cube_shader.V_loc = glGetUniformLocation(cube_shader.shader_id, "V");
	assert(cube_shader.V_loc > -1);
	cube_shader.P_loc = glGetUniformLocation(cube_shader.shader_id, "P");
	assert(cube_shader.P_loc > -1);
	

	// cube-map shaders
	sky_cube.shader = create_programme_from_files("Shaders/sky_cube.vert", "Shaders/sky_cube.frag");
	glUseProgram(sky_cube.shader);
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


	glUseProgram(cube_shader.shader_id);
	glUniformMatrix4fv(cube_shader.P_loc, 1, GL_FALSE, proj_mat.m);
	glUniformMatrix4fv(cube_shader.V_loc, 1, GL_FALSE, view_mat.m);

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


	//PARTICLE SYSTEM
	forces.push_back(new gravity(vec3(0, -4, 0)));
	forces.push_back(new drag(0.05));
	pear.push_back(new magicPear(pear_points));
	p.set_u_forces(forces);
	p.set_p_forces(pear);

	mat4 cube_model_mat = identity_mat4();
	mat4 one_deg_mat = mat4(p.cos_theta, p.sin_theta, 0, 0,
							-p.sin_theta, p.cos_theta, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1);

	//MAIN LOOP
	while (!glfwWindowShouldClose(g_window)) {
		// wipe the drawing surface clear
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render a sky-box using the cube-map texture
		sky_cube.draw();
		
		//'Particles'
		glUseProgram(simple_shader.shader_id);
		glUniformMatrix4fv(simple_shader.V_loc, 1, GL_FALSE, view_mat.m);
		p.update();
		p.locations(locations);
		for (int i = 0; i < NUM_PARTICLES; i++) {
			glUniformMatrix4fv(simple_shader.M_loc, 1, GL_FALSE, locations[i].m);
			ball.draw();
		}

		//Draw containing cube		
		glDisable(GL_CULL_FACE); // disable cull face, I think one or more of the cube faces may be backwards
		if (p.rotating) {
			cube_model_mat = one_deg_mat*cube_model_mat;
		}
		glUseProgram(cube_shader.shader_id);
		glUniformMatrix4fv(cube_shader.V_loc, 1, GL_FALSE, view_mat.m);
		glUniformMatrix4fv(cube_shader.M_loc, 1, GL_FALSE, cube_model_mat.m);
		cube.draw();
		glEnable(GL_CULL_FACE); // enable cull face

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
			delete pear_points;
		}
	}
}