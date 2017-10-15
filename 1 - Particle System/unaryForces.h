#pragma once
#include "Antons_maths_funcs.h"
#include "particle.h"

class unaryForce {
public:
	virtual vec3 apply_force(particle p) = 0;
};

class drag : public unaryForce {
	float drag_coeff;
public:
	drag() {
		drag_coeff = 0.1;
	};
	drag(float coeff) {
		drag_coeff = coeff;
	};
	vec3 apply_force(particle p) {
		return p.v*-drag_coeff;
	};
};

class gravity : public unaryForce {
	vec3 gravity_vec;
public:
	gravity() {
		gravity_vec = vec3(0.0, -1.0, 0.0);
	};
	gravity(vec3 grav_vec) {
		gravity_vec = grav_vec;
	};
	vec3 apply_force(particle p) {
		return gravity_vec*p.m;
	};
};