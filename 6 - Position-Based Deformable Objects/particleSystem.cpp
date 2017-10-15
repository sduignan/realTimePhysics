#include "particleSystem.h"

void particleSystem::setup(particle* the_particles, int particle_count, vec3* init_locations) {
	particles = the_particles;
	num_particles = particle_count;
	initial_locations = init_locations;

	timestep = 1.0 / 60;

	set_locations();
}

void particleSystem::set_u_forces(std::vector<unaryForce*> force_list){
	u_forces = force_list;
}

void particleSystem::set_p_forces(std::vector<pointForce*> force_list) {
	p_forces = force_list;
}

void particleSystem::accumulate_forces() {
	for (int i = 0; i < num_particles; i++) {
		particles[i].f = vec3(0.0, 0.0, 0.0);


		if (effect) {
			for (int j = 0; j < p_forces.size(); j++) {
				particles[i].f += p_forces[j]->apply_force(particles[i], i);
			}
		}
		else {
			for (int j = 0; j < u_forces.size(); j++) {
				particles[i].f += u_forces[j]->apply_force(particles[i]);
			}
		}
	}
}

void particleSystem::apply_forces() {
	for (int i = 0; i < num_particles; i++) {
		particles[i].v_extreme = particles[i].v + (particles[i].f / particles[i].m)*timestep;
		particles[i].x_extreme += particles[i].v_extreme*timestep;
	}
}

void particleSystem::check_planar_collisions() {
	float e = 0.8;
	float thresh = 0.0037;
	float k_f = 0.1;

	for (int i = 0; i < num_particles; i++) {
		for (int j=0; j<num_planes; j++){
			float dot_prod = dot((particles[i].x_extreme -(planes[j].normal*-dist_to_plane)), planes[j].normal);
			float v_dot_prod = dot(planes[j].normal, particles[i].v_extreme);
			if (dot_prod < thresh) {
				particles[i].x_extreme += (planes[j].normal*-dot_prod)*e;
				if (v_dot_prod < thresh) {
					float v_dot_prod = dot(particles[i].v_extreme, planes[j].normal);
					particles[i].v_extreme += (planes[j].normal * -v_dot_prod)*(1.0 + e);
				}
				float force_dot_prod = dot(planes[j].normal, particles[i].f);
				if (force_dot_prod < 0) {
					vec3 v_tan = particles[i].v_extreme - planes[j].normal * v_dot_prod;
					vec3 friction = v_tan*-k_f*-force_dot_prod;
					particles[i].f += particles[i].f*-force_dot_prod + friction;
				}
			}
		}
	}
}

void particleSystem::update_roof() {
	accumulate_forces();
	check_planar_collisions();
	apply_forces();
}

void particleSystem::update(float alpha) {
	for (int i = 0; i < num_particles; i++) {
		particles[i].v += (((particles[i].x_goal - particles[i].x_extreme) / timestep) * alpha) + (particles[i].f/particles[i].m);
		particles[i].x += particles[i].v*timestep;
	}
}

void particleSystem::non_deformable_update() {
	for (int i = 0; i < num_particles; i++) {
		particles[i].x = particles[i].x_extreme;
		particles[i].v = particles[i].v_extreme;
	}
}

void particleSystem::locations(mat4 locations[]) {
	mat4 i_mat = identity_mat4();
	for (int i = 0; i < num_particles; i++) {
		i_mat.m[12] = particles[i].x.v[0];
		i_mat.m[13] = particles[i].x.v[1];
		i_mat.m[14] = particles[i].x.v[2];
		locations[i] = i_mat;
	}
}

void particleSystem::reset() {
	set_locations();
}

void particleSystem::set_locations() {
	for (int i = 0; i < num_particles; i++) {
		particles[i].x = initial_locations[i];
		particles[i].x_extreme = initial_locations[i];
		particles[i].x_goal = initial_locations[i];
		particles[i].v = vec3(0, 0, 0);
		particles[i].v_extreme = vec3(0, 0, 0);
	}
}

void particleSystem::toggle_rotation() {
	rotate = !rotate;
}

void particleSystem::reset_normals() {
	planes[0].normal = vec3(1.0, 0.0, 0.0);
	planes[1].normal = vec3(0.0, 1.0, 0.0);
	planes[2].normal = vec3(0.0, 0.0, 1.0);
	planes[3].normal = vec3(-1.0, 0.0, 0.0);
	planes[4].normal = vec3(0.0, -1.0, 0.0);
	planes[5].normal = vec3(0.0, 0.0, -1.0);
	normals_reset = true;
}

void particleSystem::stick_particles(std::vector<int> indices) {
	for (int i = 0; i < indices.size(); i++) {
		particles[indices[i]].x = initial_locations[indices[i]];
		particles[indices[i]].x_extreme = initial_locations[indices[i]];
		particles[indices[i]].v = vec3(0, 0, 0);
		particles[indices[i]].v_extreme = vec3(0, 0, 0);
	}
}