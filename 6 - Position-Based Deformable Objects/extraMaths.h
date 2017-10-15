#pragma once
#include "Antons_maths_funcs.h"
#include <armadillo>

inline mat3 sqrt(mat3 M) {
	arma::mat arma_M(3, 3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			arma_M(i, j) = M.m[i + (j * 3)];
		}
	}

	arma::mat root_mat = arma::sqrtmat_sympd(arma_M);

	mat3 res;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++){
			res.m[i + (j * 3)] = root_mat(i, j);
		}
	}

	return res;
};

inline mat3 conserveVolume(mat3 M) {
	float det_M = determinant(M);
	float cube_root = abs(cbrt(det_M));

	return M*(1.0/cube_root);
};