#pragma once
#include "MathTypes.h"
#include "GazeEstimationTypes.h"

Vec3 calculate_q(double kq, const Vec3& o, const Vec3& u);
Vec3 calculate_cornea_center(const Vec3& q, const Vec3& light, const Vec3& camera_position, double R);
Vec3 calculate_optic_axis_unit_vector(const Vec3 pupil_center_wcs, const Vec3 center_of_cornea);
Vec3 calculate_pupil_center_wcs(const Vec3& pupil_wcs, const Vec3& camera_position,
	const Vec3& center_of_cornea, double R, double K,
	const double& n1, const double& n2);
Vec3 calculate_p(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, double K, const double& n1, const double& n2);
Vec3 calculate_iota(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, const double n1, const double n2);
double calculate_kr(const Vec3& camera_position, const Vec3& image_pupil_center,
	const Vec3& cornea_center, double R);
Vec3 calculate_r(const Vec3& camera_position, const Vec3& pupil_image_wcs,
	const Vec3& cornea_wcs, double R);
Vec3 calculate_gaze_point(const Vec3& optical_axis_unit_vector, double alpha, double beta,
	const Vec3& cornea_wcs_transfer);