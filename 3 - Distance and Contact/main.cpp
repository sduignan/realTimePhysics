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
#include "narrow_phase_display.h"

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

float increment = 1.0f;
float trans_increment = 0.1f;
float cam_heading = 0.0f; // y-rotation in degrees

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
shader_info line_shader;

int turn = 1;
int choice = 0;
int num_choices = 3;

int text_id;

NarrowPhaseDisplay proximity;

double x, y;
bool rotating = false;
bool translating = false;
float mouse_inc = 0.05f;

//Mouse control
void My_Mouse_Callback(GLFWwindow* window, double xpos, double ypos) {
	if (choice <= 1) {
		x = (10.0 - 2 * proximity.point.v[2])*(xpos / (float)g_gl_width - 0.5);
		y = -(10.0 - 2 * proximity.point.v[2])*(ypos / (float)g_gl_height - 0.5);

		vec4 point = vec4(x, y, 0, 0);
		point = inverse(view_mat)*inverse(proj_mat)*point;

		proximity.point.v[0] = point.v[0];
		proximity.point.v[1] = point.v[1];
	}
	else {
		if (rotating) {
			//Rotation
			glfwGetCursorPos(window, &x, &y);

			float horizontal_angle = mouse_inc*(x - (double)g_gl_width / 2.0);
			float vertical_angle = mouse_inc*(y - (double)g_gl_height / 2.0);

			vec4 temp;
			mat4 rotations = rotate_x_deg(rotate_y_deg(identity_mat4(), horizontal_angle), vertical_angle);
			for (int i = 0; i < 3; i++) {
				temp = rotations * vec4(proximity.triangle_b[i], 1.0);
				proximity.triangle_b[i] = vec3(temp.v[0], temp.v[1], temp.v[2]);
			}

			glfwSetCursorPos(window, (double)g_gl_width / 2.0, (double)g_gl_height / 2.0);
		}
		else if (translating){
			//Rotation
			glfwGetCursorPos(window, &x, &y);

			float horizontal_trans = 0.1*mouse_inc*(x - (double)g_gl_width / 2.0);
			float vertical_trans = -0.1*mouse_inc*(y - (double)g_gl_height / 2.0);


			vec4 temp;
			mat4 translations = translate(identity_mat4(), vec3(horizontal_trans, vertical_trans, 0.0));
			for (int i = 0; i < 3; i++) {
				temp = translations * vec4(proximity.triangle_b[i], 1.0);
				proximity.triangle_b[i] = vec3(temp.v[0], temp.v[1], temp.v[2]);
			}

			glfwSetCursorPos(window, (double)g_gl_width / 2.0, (double)g_gl_height / 2.0);

		}
	}
}

//Other Mouse Control
void Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods)
{
	if (choice <= 1) {
		return;
	}
	if (action == GLFW_PRESS) {
		glfwSetCursorPos(window, (double)g_gl_width / 2.0, (double)g_gl_height / 2.0);
		if (button == GLFW_MOUSE_BUTTON_RIGHT) {
			rotating = true;
		}
		else if (button == GLFW_MOUSE_BUTTON_LEFT) {
			translating = true;
		}
	}
	else if (action == GLFW_RELEASE) {
		rotating = false;
		translating = false;
	}	
}

//Scroll Control
void Scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	if (choice <= 1) {
		proximity.point.v[2] -= trans_increment*yoffset;
	}
	else {
		for (int i = 0; i < 3; i++) {
			proximity.triangle_b[i].v[2] -= trans_increment*yoffset;
		}
	}
}

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
		case GLFW_KEY_SPACE: {
			choice = (choice + 1) % num_choices;
			break;
		}
		case GLFW_KEY_ENTER: {
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

	//Set point size:
	glPointSize(4.0f);

	//Set line width
	glLineWidth(2.0f);

	// Tell the window where to find its key callback function
	glfwSetKeyCallback(g_window, My_Key_Callback);

	// Tell window where to find the Mouse callback function
	glfwSetCursorPosCallback(g_window, My_Mouse_Callback);

	//Cursor button callback:
	glfwSetMouseButtonCallback(g_window, Mouse_Button_Callback);
	glfwSetInputMode(g_window, GLFW_STICKY_MOUSE_BUTTONS, 1);
	glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//Scroll Control
	glfwSetScrollCallback(g_window, Scroll_callback);

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
	int tri_col_loc = glGetUniformLocation(simple_shader.shader_id, "colour");


	// line Shader
	line_shader.shader_id = create_programme_from_files("Shaders/string.vert", "Shaders/string.frag");
	glUseProgram(line_shader.shader_id);
	//get locations of M, V, P matrices
	line_shader.M_loc = glGetUniformLocation(line_shader.shader_id, "M");
	assert(line_shader.M_loc > -1);
	line_shader.V_loc = glGetUniformLocation(line_shader.shader_id, "V");
	assert(line_shader.V_loc > -1);
	line_shader.P_loc = glGetUniformLocation(line_shader.shader_id, "P");
	assert(line_shader.P_loc > -1);
	int line_col_loc = glGetUniformLocation(simple_shader.shader_id, "colour");


	proximity.setup(simple_shader.shader_id, line_shader.shader_id, tri_col_loc, line_col_loc);


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


	glUseProgram(line_shader.shader_id);
	glUniformMatrix4fv(line_shader.P_loc, 1, GL_FALSE, proj_mat.m);
	glUniformMatrix4fv(line_shader.V_loc, 1, GL_FALSE, view_mat.m);
	glUniformMatrix4fv(line_shader.M_loc, 1, GL_FALSE, identity_mat4().m);

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

		glDisable(GL_CULL_FACE);

		glUseProgram(simple_shader.shader_id);
		glUniformMatrix4fv(simple_shader.V_loc, 1, GL_FALSE, view_mat.m);

		glUseProgram(line_shader.shader_id);
		glUniformMatrix4fv(line_shader.V_loc, 1, GL_FALSE, view_mat.m);

		//ball.draw();

		float dist;
		switch(choice){
		case 0:
			dist = proximity.find_edge_point_and_draw();
			break;
		case 1:
			dist = proximity.update_and_draw();
			break;
		case 2:
			dist = proximity.t_to_t_update_and_draw();
			break;
		}

		glEnable(GL_CULL_FACE); // cull face

		std::string description = "Distance: " + std::to_string(dist);

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