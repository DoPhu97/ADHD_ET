#pragma once
#include "GazeEstimationTypes.h"
#include "OneCameraSpherical.h"
#include "ceres/ceres.h"

// Functor cho từng điểm calibration
class CalibrationFunctor {
private:
	const PupilGlint2 _pupilGlint2;
	const Vec3 _target;
	const PinholeCameraModel _camera;
	const VecPair3 _lights;
	const double _n1;
	const double _n2;

public:
	CalibrationFunctor(
		const PupilGlint2& pupilGlint2,
		const Vec3& target,
		const PinholeCameraModel& camera,
		const VecPair3& lights,
		double n1, double n2)
		: _pupilGlint2(pupilGlint2)
		, _target(target)
		, _camera(camera)
		, _lights(lights)
		, _n1(n1)
		, _n2(n2)
	{}

	bool operator()(
		const double* const kq_left,
		const double* const kq_right,
		const double* alpha,
		const double* beta,
		const double* R,
		const double* K,
		double* residual) const {

		// Chuyển đổi tọa độ glint từ ảnh sang hệ tọa độ thực tế
		VecPair3 glints_wcs;
		glints_wcs.left = _camera.ics_to_wcs(_pupilGlint2.glints.left);
		glints_wcs.right = _camera.ics_to_wcs(_pupilGlint2.glints.right);
		
		// Tính toán điểm phản xạ q và tâm giác mạc
		Vec3 q1 = calculate_q(kq_left[0], _camera.position, glints_wcs.left);
		Vec3 q2 = calculate_q(kq_right[0], _camera.position, glints_wcs.right);

		Vec3 c1 = calculate_cornea_center(q1, _lights.left, _camera.position, R[0]);
		Vec3 c2 = calculate_cornea_center(q2, _lights.right, _camera.position, R[0]);

		Vec3 cornea_center = (c1 + c2) * 0.5;
		Vec3 pupil_wcs = _camera.ics_to_wcs(_pupilGlint2.pupil);

		// Tính toán trục quang học và trục thị giác
		Vec3 optic_axis = calculate_optic_axis_unit_vector(
			pupil_wcs, _camera.position, cornea_center, R[0], K[0], _n1, _n2);
		Vec3 visual_axis = calculate_visual_axis_unit_vector(optic_axis, alpha[0], beta[0]);

		// Tính residual
		Vec3 gaze_diff = _target - visual_axis;
		residual[0] = gaze_diff[0];
		residual[1] = gaze_diff[1];
		residual[2] = gaze_diff[2];

		return true;
	}
};

// Prototype của hàm calibration
std::vector<std::vector<double>> combined_calibrate(
	const std::vector<PupilGlint2>& calibData,
	const std::vector<Vec3>& targets,
	const PinholeCameraModel& camera,
	const VecPair3& lights);
