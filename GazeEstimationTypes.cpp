#include "GazeEstimationTypes.h"
#include <cmath>

Vec3 calculate_eye_angles(const Vec3& optic_axis_unit_vector) {
	return Vec3(-1 * atan(optic_axis_unit_vector[0] / optic_axis_unit_vector[2]),
		asin(optic_axis_unit_vector[1]), 0);
}

Vec3 calculate_nu_ecs(double alpha, double beta) {
	return Vec3(-sin(alpha) * cos(beta), sin(beta), cos(alpha) * cos(beta));
}

