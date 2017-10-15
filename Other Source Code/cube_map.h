#pragma once
#include <GL/glew.h>
#include "Antons_maths_funcs.h"
#include <assert.h>
#include "stb_image.h"
#include <stdlib.h>

class cube_map
{
public:
	int point_count;
	GLuint vao;
	mat4 proj, view;
	int proj_loc, view_loc;
	GLuint tex;
	GLuint shader;

	cube_map(
		const char* front_tex,
		const char* back_tex,
		const char* top_tex,
		const char* bottom_tex,
		const char* left_tex,
		const char* right_tex
	);

	bool load_side(GLenum side_target, const char* file_name);
	void update_mats(bool P, bool V);
	void draw();
};
