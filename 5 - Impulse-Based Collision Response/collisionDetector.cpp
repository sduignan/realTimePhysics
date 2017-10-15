#include "collisionDetector.h"

void CollisionDetector::broadPhase(std::vector<RigidBody*> bodies, int num) {
	bool **potential = new bool*[num];
	for (int i = 0; i < num; i++) {
		potential[i] = new bool[num];
	}
	boundingSpheres(bodies, num, potential);
	obb(bodies, num, potential);

	for (int i = 0; i < num; i++) {
		delete[] potential[i];
	}
	delete[] potential;
}

void CollisionDetector::boundingSpheres(std::vector<RigidBody*> bodies, int num, bool **potential) {
	for (int i = 0; i < num; i++) {
		bodies[i]->colliding = false;
		for (int j = 0; j < num; j++) {
			potential[i][j] = false;
		}
	}

	for (int i = 0; i < num; i++) {
		for (int j = i + 1; j < num; j++) {
			if (length(bodies[i]->centre - bodies[j]->centre) <= bodies[i]->radius + bodies[j]->radius) {
				bodies[i]->colliding = true;
				bodies[j]->colliding = true;
				potential[i][j] = true;
			}
		}
	}
}

void CollisionDetector::obb(std::vector<RigidBody*> bodies, int num, bool **potential) {
	for (int i = 0; i < num; i++) {
		bodies[i]->OBB_colliding = false;
	}

	vec3 x(1, 0, 0);
	vec3 y(0, 1, 0);
	vec3 z(0, 0, 1);

	vec3** axes = new vec3*[bodies.size()];
	for (int i = 0; i < num; i++) {
		axes[i] = new vec3[3];
		if (bodies[i]->colliding) {
			axes[i][0] = normalise(bodies[i]->OBB_orientation*x);
			axes[i][1] = normalise(bodies[i]->OBB_orientation*y);
			axes[i][2] = normalise(bodies[i]->OBB_orientation*z);
		}
	}
	bool res;
	for (int i = 0; i < num; i++) {
		for (int j = i + 1; j < num; j++) {
			if (potential[i][j]) {
				if (obb_to_obb(*bodies[i], axes[i], *bodies[j], axes[j])) {
					bodies[i]->OBB_colliding = true;
					bodies[j]->OBB_colliding = true;
				}
				else {
					potential[i][j] = false;
				}
			}			
		}
	}


	for (int i = 0; i < num; ++i) {
		delete[] axes[i];
	}
	delete[] axes;
}

bool CollisionDetector::obb_to_obb(RigidBody a, vec3 a_axes[], RigidBody b, vec3 b_axes[]) {
	float R[3][3], AbsR[3][3];
	float ra, rb;

	for (int i = 0; i < 3; i++ ) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = dot(a_axes[i], b_axes[j]);
			AbsR[i][j] = abs(R[i][j]);
		}
	}

	vec3 temp = b.centre - a.centre;
	vec3 t = vec3(dot(temp, a_axes[0]), dot(temp, a_axes[1]), dot(temp, a_axes[2]));

	//Simplest case, along the axes of A's bounding box:
	for (int i = 0; i < 3; i++) {
		ra = a.OBB_extents.v[i];
		rb = b.OBB_extents.v[0] * AbsR[i][0] + b.OBB_extents.v[1] * AbsR[i][1] + b.OBB_extents.v[2] * AbsR[i][2];
		if (abs(t.v[i]) >= ra + rb) {
			return false;
		}
	}

	//Almost as straightforward: axes of B's bounding box:
	for (int i = 0; i < 3; i++) {
		ra = a.OBB_extents.v[0] * AbsR[0][i] + a.OBB_extents.v[1] * AbsR[1][i] + a.OBB_extents.v[2] * AbsR[2][i];
		rb = b.OBB_extents.v[i];
		if (abs(t.v[0]*R[0][i] + t.v[1]*R[1][i] + t.v[2]*R[2][i]) > ra + rb) {
			return false;
		}
	}

	//Test axis: A0xB0:
	ra = a.OBB_extents.v[1] * AbsR[2][0] + a.OBB_extents.v[2] * AbsR[1][0];
	rb = b.OBB_extents.v[1] * AbsR[0][2] + b.OBB_extents.v[2] * AbsR[0][1];
	if (abs(t.v[2] * R[1][0] - t.v[1] * R[2][0]) > ra + rb) {
		return false;
	}

	//Test axis: A0xB1:
	ra = a.OBB_extents.v[1] * AbsR[2][1] + a.OBB_extents.v[2] * AbsR[1][1];
	rb = b.OBB_extents.v[0] * AbsR[0][2] + b.OBB_extents.v[2] * AbsR[0][0];
	if (abs(t.v[2] * R[1][1] - t.v[1] * R[2][1]) > ra + rb) {
		return false;
	}
	
	//Test axis: A0xB2:
	ra = a.OBB_extents.v[1] * AbsR[2][2] + a.OBB_extents.v[2] * AbsR[1][2];
	rb = b.OBB_extents.v[0] * AbsR[0][1] + b.OBB_extents.v[1] * AbsR[0][0];
	if (abs(t.v[2] * R[1][2] - t.v[1] * R[2][2]) > ra + rb) {
		return false;
	}

	//Test axis: A1xB0:
	ra = a.OBB_extents.v[0] * AbsR[2][0] + a.OBB_extents.v[2] * AbsR[0][0];
	rb = b.OBB_extents.v[1] * AbsR[1][2] + b.OBB_extents.v[2] * AbsR[1][1];
	if (abs(t.v[0] * R[2][0] - t.v[2] * R[0][0]) > ra + rb) {
		return false;
	}
	
	//Test axis: A1xB1:
	ra = a.OBB_extents.v[0] * AbsR[2][1] + a.OBB_extents.v[2] * AbsR[0][1];
	rb = b.OBB_extents.v[0] * AbsR[1][2] + b.OBB_extents.v[2] * AbsR[1][0];
	if (abs(t.v[0] * R[2][1] - t.v[2] * R[0][1]) > ra + rb) {
		return false;
	}

	//Test axis: A1xB2:
	ra = a.OBB_extents.v[0] * AbsR[2][2] + a.OBB_extents.v[2] * AbsR[0][2];
	rb = b.OBB_extents.v[0] * AbsR[1][1] + b.OBB_extents.v[1] * AbsR[1][0];
	if (abs(t.v[0] * R[2][2] - t.v[2] * R[0][2]) > ra + rb) {
		return false;
	}

	//Test axis: A2xB0:
	ra = a.OBB_extents.v[0] * AbsR[1][0] + a.OBB_extents.v[1] * AbsR[0][0];
	rb = b.OBB_extents.v[1] * AbsR[2][2] + b.OBB_extents.v[2] * AbsR[2][1];
	if (abs(t.v[1] * R[0][0] - t.v[0] * R[1][0]) > ra + rb) {
		return false;
	}

	//Test axis: A2xB1:
	ra = a.OBB_extents.v[0] * AbsR[1][1] + a.OBB_extents.v[1] * AbsR[0][1];
	rb = b.OBB_extents.v[0] * AbsR[2][2] + b.OBB_extents.v[2] * AbsR[2][0];
	if (abs(t.v[1] * R[0][1] - t.v[0] * R[1][1]) > ra + rb) {
		return false;
	}

	//Test axis: A2xB2:
	ra = a.OBB_extents.v[0] * AbsR[1][2] + a.OBB_extents.v[1] * AbsR[0][2];
	rb = b.OBB_extents.v[0] * AbsR[2][1] + b.OBB_extents.v[1] * AbsR[2][0];
	if (abs(t.v[1] * R[0][2] - t.v[0] * R[1][2]) > ra + rb) {
		return false;
	}

	return true;
}