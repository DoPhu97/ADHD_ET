#pragma once
#include "MathTypes.h"
#include <vector>
#include "PinholeCameraModel.h"


struct PupilGlint2 {
	Vec2 pupil;
	VecPair2 glints;
};

struct PupilGlints2 {
	std::vector<Vec2> pupil;
	std::vector<VecPair2> glints;
};

struct Eyes2 {
	PupilGlint2 eyeL;
	PupilGlint2 eyeR;
};

struct EyeParameter
{
	// Eye parameters
	double alpha;
	double beta;
	double R; // R in cm
	double K; // K in cm
	double n1;
	double n2;
	double D; // D in cm
};

inline Vec3 calculate_eye_angles(const Vec3& optic_axis_unit_vector)
{
	return Vec3(-1 * atan(optic_axis_unit_vector[0] / optic_axis_unit_vector[2]),
		asin(optic_axis_unit_vector[1]), 0);
}

inline Vec3 calculate_nu_ecs(double alpha, double beta)
{
	return Vec3(-sin(alpha) * cos(beta), sin(beta), cos(alpha) * cos(beta));
}


 /// Note that this is not equal to a general rotation matrix, as the coordinate system this 
/// transforms from is flipped as well.
inline Mat3x3 calculate_eye_rotation_matrix(double theta, double phi, double kappa)
{
	Mat3x3 Rflip, Rtheta, Rphi, Rkappa;
	Rflip << -1, 0, 0,
		0, 1, 0,
		0, 0, -1;
	Rtheta << std::cos(theta), 0, -std::sin(theta),
		0, 1, 0,
		std::sin(theta), 0, std::cos(theta);
	Rphi << 1, 0, 0,
		0, std::cos(phi), std::sin(phi),
		0, -std::sin(phi), std::cos(phi);
	Rkappa << std::cos(kappa), -std::sin(kappa), 0,
		std::sin(kappa), std::cos(kappa), 0,
		0, 0, 1;

	return mat_prod(Rflip, mat_prod(Rtheta, mat_prod(Rphi, Rkappa)));

}
