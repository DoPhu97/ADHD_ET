#pragma once
#include "MathTypes.h"
#include <vector>

struct PupilGlint2 {
	Vec2 pupil;
	VecPair2 glints;
};

struct EyeParameter {
	double alpha;
	double beta;
	double R; // Radius of cornea (mm)
	double K; // Distance from cornea center to pupil (mm)
	double n1; // Refractive index of cornea
	double n2; // Refractive index of aqueous humor
};

Vec3 calculate_eye_angles(const Vec3& optic_axis_unit_vector);
Vec3 calculate_nu_ecs(double alpha, double beta);
