#include "particleSystem.h"

particleSystem::particleSystem() {
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
	for (int i = 0; i < NUM_PARTICLES; i++) {
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
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles[i].x += particles[i].v*timestep;
		particles[i].v += (particles[i].f / particles[i].m)*timestep;
	}
}

void particleSystem::check_planar_collisions() {
	float e = 0.8;
	float thresh = 0.0037;
	float k_f = 0.1;
	
	if (rotate || (deg % 90 != 0)) {
		rotating = true;
		normals_reset = false;
		for (int i = 0; i < NUM_PLANES; i++) {
			plane_normals[i].v[0] = cos_theta*plane_normals[i].v[0] - sin_theta*plane_normals[i].v[1];
			plane_normals[i].v[1] = sin_theta*plane_normals[i].v[0] + cos_theta*plane_normals[i].v[1];
			plane_normals[i] = normalise(plane_normals[i]);
		}
		deg++;
	}
	else {
		rotating = false;
		if (!normals_reset) {
			reset_normals();
		}
	}


	for (int i = 0; i < NUM_PARTICLES; i++) {
		for (int j=0; j<NUM_PLANES; j++){
			float dot_prod = dot((particles[i].x -(plane_normals[j]*-dist_to_plane)), plane_normals[j]);
			float v_dot_prod = dot(plane_normals[j], particles[i].v);
			if (dot_prod < thresh) {
				particles[i].x += (plane_normals[j]*-dot_prod)*e;
				if (v_dot_prod < thresh) {
					float v_dot_prod = dot(particles[i].v, plane_normals[j]);
					particles[i].v += (plane_normals[j] * -v_dot_prod)*(1.0 + e);
				}
				float force_dot_prod = dot(plane_normals[j], particles[i].f);
				if (force_dot_prod < 0) {
					vec3 v_tan = particles[i].v - plane_normals[j] * v_dot_prod;
					vec3 friction = v_tan*-k_f*-force_dot_prod;
					particles[i].f += particles[i].f*-force_dot_prod + friction;
				}
			}
		}
	}
}

void particleSystem::update() {
	accumulate_forces();
	check_planar_collisions();
	apply_forces();
}

void particleSystem::locations(mat4 locations[]) {
	mat4 i_mat = identity_mat4();
	for (int i = 0; i < NUM_PARTICLES; i++) {
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
	for (int i = 0; i < 30; i++) {
		for (int j = 0; j < 40; j++) {
			particles[i * 40 + j].x = vec3(((float)j*0.1) - 2.0, 2.0, ((float)i*0.1) - 1.5);
			particles[i * 40 + j].v = vec3(0.0, 0.0, 0.0);
			particles[i * 40 + j].m = (float)(rand() % 1000 + 100) / 1000.0;
		}
	}
}

void particleSystem::toggle_rotation() {
	rotate = !rotate;
}

void particleSystem::reset_normals() {
	plane_normals[0] = vec3(1.0, 0.0, 0.0);
	plane_normals[1] = vec3(0.0, 1.0, 0.0);
	plane_normals[2] = vec3(0.0, 0.0, 1.0);
	plane_normals[3] = vec3(-1.0, 0.0, 0.0);
	plane_normals[4] = vec3(0.0, -1.0, 0.0);
	plane_normals[5] = vec3(0.0, 0.0, -1.0);
	normals_reset = true;
}