#pragma once
#include "MathTypes.h"
#include "PinholeCameraModel.h"
#include "GazeEstimationTypes.h"
#include "OneCameraSpherical.h"
#include <ceres/ceres.h>
#include <vector>




class UnifiedCalibrationFunctor {
private:
	const PupilGlint2 _pupilGlint;
	const Vec3 _target;
	const PinholeCameraModel _camera;
	const VecPair3 _lights;
	const double _n1, _n2;
	const Vec3 _screen_center;
	const Mat3x3 _matrix;

public:
	UnifiedCalibrationFunctor(const PupilGlint2& pupilGlint, const Vec3& target,
		const PinholeCameraModel& camera, const VecPair3& lights,
		double n1, double n2, const Vec3& screen_center, const Mat3x3& matrix);
	bool operator()(const double* const alpha, const double* const beta,
		const double* const R, const double* const K,
		const double* const k_left, const double* const k_right,
		double* residual) const;
};

std::vector<std::vector<double>> calibrate(
	const std::vector<std::vector<PupilGlint2>>& calibData,
	const std::vector<Vec3>& targets,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const Vec3 screen_center,
	const Mat3x3 rotation_matrix);