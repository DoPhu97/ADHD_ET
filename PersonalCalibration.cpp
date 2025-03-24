#include "PersonalCalibration.h"
#include <iostream>



UnifiedCalibrationFunctor::UnifiedCalibrationFunctor(
	const PupilGlint2& pupilGlint, 
	const Vec3& target,
	const PinholeCameraModel& camera, 
	const VecPair3& lights,
	double n1, double n2, 
	const Vec3& screen_center, const Mat3x3& matrix)
	: _pupilGlint(pupilGlint), _target(target), _camera(camera), _lights(lights),
	_n1(n1), _n2(n2), _screen_center(screen_center), _matrix(matrix) {}

bool UnifiedCalibrationFunctor::operator()(const double* const alpha, const double* const beta,
	const double* const R, const double* const K,
	const double* const k_left, const double* const k_right,
	double* residual) const {
	Vec3 q1 = calculate_q(k_left[0], _camera.position, _camera.ics_to_wcs(_pupilGlint.glints.left));
	Vec3 q2 = calculate_q(k_right[0], _camera.position, _camera.ics_to_wcs(_pupilGlint.glints.right));
	Vec3 c1 = calculate_cornea_center(q1, _lights.left, _camera.position, R[0]);
	Vec3 c2 = calculate_cornea_center(q2, _lights.right, _camera.position, R[0]);
	Vec3 cornea_wcs = (c1 + c2) * 0.5;

	Vec3 pupil_img_wcs = _camera.ics_to_wcs(_pupilGlint.pupil);

	Vec3 pupil_wcs = calculate_pupil_center_wcs(pupil_img_wcs, _camera.position,
		cornea_wcs, R[0], K[0], _n1, _n2);

	Vec3 optic_axis = calculate_optic_axis_unit_vector(
		_matrix *(pupil_wcs),
		_matrix * (cornea_wcs));

	Vec3 gaze_point = calculate_gaze_point(optic_axis, alpha[0], beta[0],
		_matrix * (cornea_wcs - _screen_center));

	Vec3 diff = _target - gaze_point;
	residual[0] = diff[0];
	residual[1] = diff[1];
	residual[2] = diff[2];
	return true;
}

std::vector<std::vector<double>> calibrate(
    const std::vector<std::vector<PupilGlint2>>& calibData,
    const std::vector<Vec3>& targets,
    const PinholeCameraModel& camera,
    const VecPair3& lights,
    const Vec3 screen_center,
    const Mat3x3 rotation_matrix) {

    if (calibData.size() != targets.size()) {
        std::cerr << "Error: Number of stimulus points and targets mismatch!" << std::endl;
        return {};
    }

    const int n = calibData.size();
    double alpha = deg_to_rad(4.5);
    double beta = deg_to_rad(2.5);
    double R = 7.0;
    double K = 4.0;
    std::vector<double> k_left(n, 500.0);
    std::vector<double> k_right(n, 500.0);

    ceres::Problem problem;
    

    for (int i = 0; i < n; ++i) {
        const auto& group = calibData[i];
        if (group.empty()) {
            std::cerr << "Error: Empty data group at index " << i << "!" << std::endl;
            return {};
        }
        for (const auto& data : group) {
            // Residual block cho mục tiêu gaze chính
            auto* cost_function = new ceres::NumericDiffCostFunction
                <UnifiedCalibrationFunctor, ceres::CENTRAL, 3, 1, 1, 1, 1, 1, 1>
                (new UnifiedCalibrationFunctor(data, targets[i], camera, lights,
                    1.3375, 1.0, screen_center, rotation_matrix));
            problem.AddResidualBlock(cost_function, nullptr,
                &alpha, &beta, &R, &K, &k_left[i], &k_right[i]);
               
        }
    }

    problem.SetParameterLowerBound(&alpha, 0, deg_to_rad(1));
    problem.SetParameterUpperBound(&alpha, 0, deg_to_rad(5));
    problem.SetParameterLowerBound(&beta, 0, deg_to_rad(1));
    problem.SetParameterUpperBound(&beta, 0, deg_to_rad(5));
    problem.SetParameterLowerBound(&R, 0, 7.0);
    problem.SetParameterUpperBound(&R, 0, 9.0);
    problem.SetParameterLowerBound(&K, 0, 4.0);
    problem.SetParameterUpperBound(&K, 0, 5.0);
    for (int i = 0; i < n; ++i) {
        problem.SetParameterLowerBound(&k_left[i], 0, 300.0);
        problem.SetParameterUpperBound(&k_left[i], 0, 900.0);
        problem.SetParameterLowerBound(&k_right[i], 0, 300.0);
        problem.SetParameterUpperBound(&k_right[i], 0, 900.0);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::vector<std::vector<double>> result = { {alpha}, {beta}, {R}, {K} };
    for (int i = 0; i < n; ++i) {
        result.push_back({ k_left[i], k_right[i] });
    }
    return result;
}