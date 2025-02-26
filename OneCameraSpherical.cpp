#pragma once
#include <vector>
#include "MathTypes.h"
#include "GazeEstimationTypes.h"
#include "OneCameraSpherical.h"

Vec3 calculate_q(double kq, const Vec3& o, const Vec3& u) {
	return o + kq * (o - u).normalized();
}

Vec3 calculate_cornea_center(const Vec3& q, const Vec3& light,
	const Vec3& camera_position, const double R)
{
	const Vec3 l_q_unit = (light - q).normalized();
	const Vec3 o_q_unit = (camera_position - q).normalized();
	return q - R * (l_q_unit + o_q_unit).normalized();
}

Vec3 calculate_optic_axis_unit_vector(const Vec3& pupil_wcs, const Vec3& camera_position,
	const Vec3& center_of_cornea, double R, double K, double n1, double n2)
{
	const Vec3 pupil_por_wcs = calculate_r(camera_position, pupil_wcs, center_of_cornea, R);
	Vec3 pupil_center_wcs = calculate_p(camera_position, pupil_por_wcs, center_of_cornea, R, K, n1, n2);
	return normalized(pupil_center_wcs - center_of_cornea);
}

Vec3 calculate_p(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, double K, double n1, double n2)
{
	Vec3 iota = calculate_iota(camera_position, pupil_por_wcs, center_of_cornea, R, n1, n2);
	double rc_dot_iota = dot((pupil_por_wcs - center_of_cornea), iota);
	double sqrt_term = rc_dot_iota * rc_dot_iota - (R * R - K * K);
	double kp = -1 * rc_dot_iota - sqrt(rc_dot_iota*rc_dot_iota - (R * R - K * K));
	return pupil_por_wcs + kp * iota;
}

Vec3 calculate_iota(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, double n1, double n2)
{
	Vec3 zeta = normalized(camera_position - pupil_por_wcs);
	Vec3 eta = (pupil_por_wcs - center_of_cornea) / R;
	double eta_dot_zeta = dot(eta, zeta);

	double a = eta_dot_zeta - sqrt((n1 / n2)*(n1 / n2) - 1 + eta_dot_zeta * eta_dot_zeta);
	return (n2 / n1) * (a * eta - zeta);
}

double calculate_kr(const Vec3& camera_position, const Vec3& image_pupil_center,
	const Vec3& cornea_center, double R)
{
	const double a = squared_length(camera_position - image_pupil_center);
	const double b = dot(camera_position - image_pupil_center, camera_position - cornea_center);
	const double c = squared_length(camera_position - cornea_center) - R * R;

	return (-b - sqrt(b * b - a * c)) / a;
}

Vec3 calculate_r(const Vec3& camera_position, const Vec3& pupil_image_wcs,
	const Vec3& cornea_wcs, double R)
{
	const double kr = calculate_kr(camera_position, pupil_image_wcs, cornea_wcs, R);
	return camera_position + kr * (camera_position - pupil_image_wcs);
}

Vec3 calculate_visual_axis_unit_vector(const Vec3& optical_axis_unit_vector, double alpha, double beta)
{
	const Vec3 nu_ecs = calculate_nu_ecs(alpha, beta);
	Vec3 eye_angles = calculate_eye_angles(optical_axis_unit_vector);
	const Mat3x3 Reye = calculate_eye_rotation_matrix(eye_angles[0], eye_angles[1], eye_angles[2]);
	return mat3vec3_prod(Reye, nu_ecs);
}

