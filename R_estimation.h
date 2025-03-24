#pragma once
#include "MathTypes.h"
#include "GazeEstimationTypes.h"
#include "PinholeCameraModel.h"
#include "OneCameraSpherical.h"
#include "ceres/ceres.h"

class RFunctor {
private:
	const PupilGlint2 _pupilGlint;
	const PinholeCameraModel _camera;
	const VecPair3 _lights;
	const Vec3 _cornea_truth;
	const double _R;  // R là hằng số, không phải tham số tối ưu hóa

public:
	RFunctor(const PupilGlint2& pupilGlint,
		const PinholeCameraModel& camera,
		const VecPair3& lights,
		const Vec3& cornea_truth,
		const double R)  // R truyền vào như hằng số
		: _pupilGlint(pupilGlint),
		_camera(camera),
		_lights(lights),
		_cornea_truth(cornea_truth),
		_R(R)
	{}

	bool operator()(const double* const k_left,
		const double* const k_right,
		double* residual) const
	{
		Vec3 glint_left_ccs = _camera.ics_to_ccs(_pupilGlint.glints.left);
		Vec3 glint_right_ccs = _camera.ics_to_ccs(_pupilGlint.glints.right);
		Vec3 q1 = calculate_q(k_left[0], _camera.position, glint_left_ccs);
		Vec3 q2 = calculate_q(k_right[0], _camera.position, glint_right_ccs);
		Vec3 c1 = calculate_cornea_center(q1, _lights.left, _camera.position, _R);
		Vec3 c2 = calculate_cornea_center(q2, _lights.right, _camera.position, _R);
		Vec3 cornea_ccs = (c1 + c2) * 0.5;
		Vec3 diff = _cornea_truth - cornea_ccs;
		residual[0] = diff[0];
		residual[1] = diff[1];
		residual[2] = diff[2];
		return true;
	}
};

class R_Optimization_Functor {
private:
	const PupilGlint2 _pupilGlint;
	const PinholeCameraModel _camera;
	const VecPair3 _lights;
	const Vec3 _cornea_truth;
	const double _k_left;  // k_left cố định từ giai đoạn 1
	const double _k_right; // k_right cố định từ giai đoạn 1

public:
	R_Optimization_Functor(const PupilGlint2& pupilGlint,
		const PinholeCameraModel& camera,
		const VecPair3& lights,
		const Vec3& cornea_truth,
		const double k_left,
		const double k_right)
		: _pupilGlint(pupilGlint),
		_camera(camera),
		_lights(lights),
		_cornea_truth(cornea_truth),
		_k_left(k_left),
		_k_right(k_right)
	{}

	bool operator()(const double* const R,
		double* residual) const
	{
		Vec3 glint_left_ccs = _camera.ics_to_ccs(_pupilGlint.glints.left);
		Vec3 glint_right_ccs = _camera.ics_to_ccs(_pupilGlint.glints.right);
		Vec3 q1 = calculate_q(_k_left, _camera.position, glint_left_ccs);
		Vec3 q2 = calculate_q(_k_right, _camera.position, glint_right_ccs);
		Vec3 c1 = calculate_cornea_center(q1, _lights.left, _camera.position, R[0]);
		Vec3 c2 = calculate_cornea_center(q2, _lights.right, _camera.position, R[0]);
		Vec3 cornea_ccs = (c1 + c2) * 0.5;
		Vec3 diff = _cornea_truth - cornea_ccs;
		residual[0] = diff[0];
		residual[1] = diff[1];
		residual[2] = diff[2];
		return true;
	}
};

std::vector<std::vector<double>> calculate_k_values(
	const std::vector<std::vector<PupilGlint2>>& calibData,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const std::vector<Vec3>& cornea_truth,
	const double fixed_R);  // R là hằng số truyền vào

double calculate_final_R(
	const std::vector<std::vector<PupilGlint2>>& calibData,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const std::vector<Vec3>& cornea_truth,
	const std::vector<std::vector<double>>& k_values);
