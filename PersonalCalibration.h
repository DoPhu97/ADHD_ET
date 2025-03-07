#pragma once
#include "GazeEstimationTypes.h"
#include "OneCameraSpherical.h"
#include "ceres/ceres.h"

// Declare calculate_cornea_center_wcs before its usage
Vec3 calculate_cornea_center_wcs(
	const PupilGlint2 pupilGlint2,
	const VecPair3 lights,
	const PinholeCameraModel camera,
	double R);


class CorneaDistanceFunctor {
private:
	const PupilGlint2 _pupilGlint2;
	const VecPair3 _lights;
	const PinholeCameraModel _camera;
	double _R;

public:
	CorneaDistanceFunctor(
		const PupilGlint2& pupilGlint2,
		const PinholeCameraModel& camera,
		const VecPair3& lights,
		double R)
		: _pupilGlint2(pupilGlint2), _camera(camera), _lights(lights), _R(R)
	{}

	bool operator()(
		const double* const kq_left,
		const double* const kq_right,
		double* residual) const {
		Vec3 q1 = calculate_q(kq_left[0], _camera.position, _camera.ics_to_wcs(_pupilGlint2.glints.left));
		Vec3 q2 = calculate_q(kq_right[0], _camera.position, _camera.ics_to_wcs(_pupilGlint2.glints.right));
		Vec3 c1 = calculate_cornea_center(q1, _lights.left, _camera.position, _R);
		Vec3 c2 = calculate_cornea_center(q2, _lights.right, _camera.position, _R);
		residual[0] = c1[0] - c2[0];
		residual[1] = c1[1] - c2[1];
		residual[2] = c1[2] - c2[2];
		return true;
	}
};

class PersonalCalibrationFunctor {
private:
	const PupilGlint2 _pupilGlint2;
	const Vec3 _target;
	const PinholeCameraModel _camera;
	const VecPair3 _lights;
	const double _n1;
	const double _n2;
	const Vec3 _screen_center;
	const Mat3x3 _rotation_matrix;

public:
	PersonalCalibrationFunctor(
		const PupilGlint2& pupilGlint2,
		const Vec3& target,
		const PinholeCameraModel& camera,
		const VecPair3& lights,
		double n1, double n2,
		const Vec3 screen_center,
		const Mat3x3 rotation_matrix)
		:
		_pupilGlint2(pupilGlint2), 
		_target(target), 
		_camera(camera), 
		_lights(lights), 
		_n1(n1), _n2(n2),
		_screen_center(screen_center),
		_rotation_matrix(rotation_matrix)
	{}

	bool operator()(
		const double* const alpha,
		const double* const beta,
		const double* const R,
		const double* const K,
		double* residual) const 
	{
		Vec3 pupil_img_wcs = _camera.ics_to_wcs(_pupilGlint2.pupil);

		Vec3 cornea_wcs = calculate_cornea_center_wcs(_pupilGlint2, _lights, _camera, R[0]);

		Vec3 pupil_wcs = calculate_pupil_center_wcs(pupil_img_wcs, _camera.position,
												cornea_wcs, R[0], K[0], _n1, _n2);

		//Transfer to screen
		Vec3 cornea_wcs_transfer = _rotation_matrix* (cornea_wcs - _screen_center);

		Vec3 optic_axis = calculate_optic_axis_unit_vector(
						_rotation_matrix*(pupil_wcs - _screen_center),
						cornea_wcs_transfer);

		
		Vec3 gaze_diff = _target - calculate_gaze_point(optic_axis, 
									alpha[0], beta[0], cornea_wcs_transfer);

		residual[0] = gaze_diff[0];
		residual[1] = gaze_diff[1];
		residual[2] = gaze_diff[2];

		return true;
	}
};


std::vector<std::vector<double>> calibrate(
	const std::vector<PupilGlint2>& calibData,
	const std::vector<Vec3>& targets,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const Vec3 screen_center,
	const Mat3x3 rotation_matrix);
