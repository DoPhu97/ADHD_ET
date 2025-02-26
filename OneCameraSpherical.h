#pragma once
#include <vector>
#include "ceres/ceres.h"
#include "GazeEstimationTypes.h"

Vec3 calculate_q(double kq, const Vec3& o, const Vec3& u);

Vec3 calculate_cornea_center(const Vec3& q, const Vec3& light,
	const Vec3& camera_position, const double R);

Vec3 calculate_cornea_center_wcs(const VecPair2& glints, const EyeParameter& eyePar,
	const VecPair3& lights, const PinholeCameraModel& camera);

Vec3 gaze_estimation(const PupilGlint2& pupilGlint2, const EyeParameter& eyePar,
	const PinholeCameraModel& camera, const VecPair3& lights);

Vec3 calculate_optic_axis_unit_vector(const Vec3& pupil_wcs, const Vec3& camera_position,
	const Vec3& center_of_cornea, double R, double K, double n1, double n2);

Vec3 calculate_p(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, double K, double n1, double n2);

Vec3 calculate_iota(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, double n1, double n2);

double calculate_kr(const Vec3& camera_position, const Vec3& image_pupil_center,
	const Vec3& cornea_center, double R);

Vec3 calculate_r(const Vec3& camera_position, const Vec3& pupil_image_wcs,
	const Vec3& cornea_wcs, double R);

Vec3 calculate_visual_axis_unit_vector(const Vec3& optical_axis_unit_vector,
	double alpha, double beta);

Vec3 calculate_nu_ecs(double alpha, double beta);

Mat3x3 calculate_eye_rotation_matrix(double theta, double phi, double kappa);

Vec3 calculate_eye_angles(const Vec3& optic_axis_unit_vector);