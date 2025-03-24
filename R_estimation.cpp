#include "R_estimation.h"



std::vector<std::vector<double>> calculate_k_values(
	const std::vector<std::vector<PupilGlint2>>& calibData,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const std::vector<Vec3>& cornea_truth,
	const double fixed_R)  // R là hằng số truyền vào
{
	const int n = calibData.size();  // 10 điểm hiệu chỉnh
	std::vector<double> k_left(n, 500.0);  // Khởi tạo k_left
	std::vector<double> k_right(n, 500.0); // Khởi tạo k_right

	ceres::Problem problem;

	for (int i = 0; i < n; ++i) {
		const auto& group = calibData[i];
		if (group.empty()) {
			std::cerr << "Error: Empty data group at index " << i << "!" << std::endl;
			return {};
		}
		for (const auto& data : group) {
			auto* cost_function = new ceres::NumericDiffCostFunction
				<RFunctor, ceres::CENTRAL, 3, 1, 1>  // 3 residuals, 2 tham số (k_left, k_right)
				(new RFunctor(data, camera, lights, cornea_truth[i], fixed_R));
			problem.AddResidualBlock(cost_function, nullptr, &k_left[i], &k_right[i]);
		}
		// Giới hạn cho k_left và k_right
		problem.SetParameterLowerBound(&k_left[i], 0, 400.0);
		problem.SetParameterUpperBound(&k_left[i], 0, 700.0);
		problem.SetParameterLowerBound(&k_right[i], 0, 400.0);
		problem.SetParameterUpperBound(&k_right[i], 0, 700.0);
	}

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 1000;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << "Phase 1 Summary (R fixed at " << fixed_R << " mm):\n" << summary.FullReport() << "\n";

	std::vector<std::vector<double>> result;
	for (int i = 0; i < n; ++i) {
		result.push_back({ k_left[i], k_right[i] });
	}
	return result;
}



double calculate_final_R(
	const std::vector<std::vector<PupilGlint2>>& calibData,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const std::vector<Vec3>& cornea_truth,
	const std::vector<std::vector<double>>& k_values)
{
	double R = 7.8;  // Giá trị khởi tạo cho R
	ceres::Problem problem;

	const int n = calibData.size();
	for (int i = 0; i < n; ++i) {
		const auto& group = calibData[i];
		if (group.empty()) {
			std::cerr << "Error: Empty data group at index " << i << "!" << std::endl;
			return -1.0;
		}
		for (const auto& data : group) {
			auto* cost_function = new ceres::NumericDiffCostFunction
				<R_Optimization_Functor, ceres::CENTRAL, 3, 1>
				(new R_Optimization_Functor(data, camera, lights, cornea_truth[i], k_values[i][0], k_values[i][1]));
			problem.AddResidualBlock(cost_function, nullptr, &R);
		}
	}

	problem.SetParameterLowerBound(&R, 0, 7.0);
	problem.SetParameterUpperBound(&R, 0, 9.0);

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 1000;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << "Phase 2 Summary:\n" << summary.FullReport() << "\n";

	return R;
}