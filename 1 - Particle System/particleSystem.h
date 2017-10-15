#pragma once
#include "Antons_maths_funcs.h"
#include <stdlib.h> //rand
#include <vector>
#include "particle.h"
#include "unaryForces.h"
#include "pointForces.h"

#define NUM_PARTICLES  1200
#define NUM_PLANES 6

class particleSystem {
private:
	particle particles[NUM_PARTICLES];
	float timestep;
	std::vector<unaryForce*> u_forces;
	std::vector<pointForce*> p_forces;
	float dist_to_plane = 2.0;
	vec3 plane_normals[NUM_PLANES] = { vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0),  vec3(0, 0, 1.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, -1.0, 0.0),  vec3(0, 0, -1.0) };

	int deg = 0;
	bool rotate = false;
	bool normals_reset = false;

public:
	bool effect = false;
	bool rotating = false;
	float cos_theta = 0.99984769515;
	float sin_theta = 0.01745240643;

public:
	particleSystem();
	void set_u_forces(std::vector<unaryForce*> force_list);
	void set_p_forces(std::vector<pointForce*> force_list);

	void toggle_rotation();
	void update();
	void locations(mat4 locations_array[]);
	void reset();

private:
	void accumulate_forces();
	void apply_forces();
	void check_planar_collisions();
	void set_locations();
	void reset_normals();
};
