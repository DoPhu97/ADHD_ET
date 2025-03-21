#include "OneCameraSpherical.h"
#include <cmath>

Vec3 calculate_q(double kq, const Vec3& o, const Vec3& u) {
	return o + kq * normalized(o - u);
}

Vec3 calculate_cornea_center(const Vec3& q, const Vec3& light, const Vec3& camera_position, double R) {
	Vec3 l_q_unit = normalized(light - q);
	Vec3 o_q_unit = normalized(camera_position - q);
	return q - R * normalized(l_q_unit + o_q_unit);
}

Vec3 calculate_optic_axis_unit_vector(const Vec3 pupil_center_wcs, const Vec3 center_of_cornea) {
	return normalized(pupil_center_wcs - center_of_cornea);
}

Vec3 calculate_pupil_center_wcs(const Vec3& pupil_wcs, 
	const Vec3& camera_position,
	const Vec3& center_of_cornea, double R, double K,
	const double& n1, const double& n2) 
{
	Vec3 pupil_por_wcs = calculate_r(camera_position, pupil_wcs, center_of_cornea, R);
	return calculate_p(camera_position, pupil_por_wcs, center_of_cornea, R, K, n1, n2);
}

Vec3 calculate_p(const Vec3& camera_position, 
	const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, 
	double R, double K, 
	const double& n1, const double& n2) 
{
	Vec3 iota = calculate_iota(camera_position, pupil_por_wcs, center_of_cornea, R, n1, n2);
	double rc_dot_iota = dot((pupil_por_wcs - center_of_cornea), iota);
	double discriminant = rc_dot_iota * rc_dot_iota - (R * R - K * K);

	double kp = -rc_dot_iota - sqrt(discriminant);
	return pupil_por_wcs + kp * iota;
}

Vec3 calculate_iota(const Vec3& camera_position, const Vec3& pupil_por_wcs,
	const Vec3& center_of_cornea, double R, const double n1, const double n2) {
	Vec3 zeta = normalized(camera_position - pupil_por_wcs);
	Vec3 eta = (pupil_por_wcs - center_of_cornea) / R;
	double eta_dot_zeta = dot(eta, zeta);
	double a = eta_dot_zeta - sqrt((n1 / n2) * (n1 / n2) - 1 + eta_dot_zeta * eta_dot_zeta);
	return (n2 / n1) * (a * eta - zeta);
}

double calculate_kr(const Vec3& camera_position, const Vec3& image_pupil_center,
	const Vec3& cornea_center, double R) {
	double a = squared_length(camera_position - image_pupil_center);
	double b = dot(camera_position - image_pupil_center, camera_position - cornea_center);
	double c = squared_length(camera_position - cornea_center) - R * R;
	return (-b - sqrt(b * b - a * c)) / a;
}

Vec3 calculate_r(const Vec3& camera_position, const Vec3& pupil_image_wcs,
	const Vec3& cornea_wcs, double R) {
	double kr = calculate_kr(camera_position, pupil_image_wcs, cornea_wcs, R);
	return camera_position + kr * (camera_position - pupil_image_wcs);
}

Vec3 calculate_gaze_point(const Vec3& optical_axis_unit_vector, double alpha, double beta,
	const Vec3& cornea_wcs_transfer) {
	Vec3 angles = calculate_eye_angles(optical_axis_unit_vector);
	double z = -cos(angles[1] + beta) * cos(angles[0] + alpha);
	Vec3 visual_axis = Vec3(cos(angles[1] + beta) * sin(angles[0] + alpha),
		sin(angles[1] + beta), z);
	double kg = -cornea_wcs_transfer[2] / z;
	return cornea_wcs_transfer + kg * visual_axis;
}