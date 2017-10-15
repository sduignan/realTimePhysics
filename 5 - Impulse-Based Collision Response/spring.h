#pragma once
#include "Antons_maths_funcs.h"

class Spring {
public:
	float k_spring;	//spring coefficient
	float zeta;	//damping coefficient
	vec3 spring_origin;
	float resting_length;
	int spring_attach_vertex_index;

	Spring() {};
	Spring(float k, float z, vec3 o, float l, int i) {
		k_spring = k;
		zeta = z;
		spring_origin = o;
		resting_length = l;
		spring_attach_vertex_index = i;
	}
};
