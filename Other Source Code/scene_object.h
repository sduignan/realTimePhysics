#pragma once
#include "Antons_maths_funcs.h"
#include <GL/glew.h>
#include "obj_parser.h"
#include <stdlib.h>
#include <assert.h>

//#define STB_IMAGE_IMPLEMENTATION

#include "stb_image.h"

//#define STBI_ASSERT(x)

class scene_object
{
public:
	int point_count;
	GLuint vao;
	mat4 model, proj, view;
	int M_loc, P_loc, V_loc;
	GLuint tex_handle, normal_handle;
	bool normal_mapped = false;
	
	scene_object();
	scene_object(char* mesh_name, char* tex_name);
	scene_object(int in_point_count, GLfloat* points, GLfloat* normals, GLfloat* tex_coords, char* tex_name);
	scene_object(char* mesh_name, char* tex_name, char* normal_map_file);

	void operator=(scene_object &source);

	void set_mat_locs(int in_M_loc, int in_P_loc, int in_V_loc);
	void update_mats(bool M, bool P, bool V);

	void draw();
	
	void fill_vao_vbo(int point_count, GLfloat* vp, GLfloat* vn, GLfloat* vt, GLuint& vao, GLfloat* vtan=NULL);
	bool load_texture (const char* file_name, GLuint* tex);	
};
