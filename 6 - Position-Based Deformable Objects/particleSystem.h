#pragma once
#include "Antons_maths_funcs.h"
#include <stdlib.h> //rand
#include <vector>
#include "particle.h"
#include "unaryForces.h"
#include "pointForces.h"

struct plane {
	vec3 normal;
	float dist;
};

class particleSystem {
private:
	particle *particles;
	int num_particles;
	float timestep;
	float dist_to_plane = 2.0;
	int num_planes = 6;
	plane planes[6] = { {vec3(1.0, 0.0, 0.0), dist_to_plane}, {vec3(0.0, 1.0, 0.0), dist_to_plane }, {vec3(0, 0, 1.0), dist_to_plane }, {vec3(-1.0, 0.0, 0.0), dist_to_plane },{vec3(0.0, -1.0, 0.0), dist_to_plane}, { vec3(0, 0, -1.0), dist_to_plane }};
	vec3* initial_locations;


	int deg = 0;
	bool rotate = false;
	bool normals_reset = false;

public:
	std::vector<unaryForce*> u_forces;
	std::vector<pointForce*> p_forces;
	bool effect = false;
	bool rotating = false;
	float cos_theta = 0.99984769515;
	float sin_theta = 0.01745240643;

public:
	particleSystem() {};
	void setup(particle* the_particles, int particle_count, vec3* init_locations);
	void set_u_forces(std::vector<unaryForce*> force_list);
	void set_p_forces(std::vector<pointForce*> force_list);

	void toggle_rotation();
	void update_roof();
	void update(float alpha);
	void non_deformable_update();
	void locations(mat4 locations_array[]);
	void reset();
	void stick_particles(std::vector<int> indices);

	void accumulate_forces();
	void apply_forces();
	void check_planar_collisions();
	void set_locations();
	void reset_normals();
};
