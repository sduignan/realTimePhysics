#pragma once
#include "Antons_maths_funcs.h"
#include "particle.h"

class pointForce {
public:
	virtual vec3 apply_force(particle p, int index) = 0;
};

class magicPear : public pointForce {
	vec3* pear_points;
public:
	magicPear(vec3* points) {
		pear_points = points;
	};
	vec3 apply_force(particle p, int index) {
		vec3 diff = pear_points[index] - p.x;
		vec3 dir = normalise(diff);
		float dist = length(diff);
		return dir*p.m*dist*5 + p.v*-(1/(dist+0.1));
	};
};