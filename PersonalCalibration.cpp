#include "PersonalCalibration.h"
#include <ceres/ceres.h>
#include <iostream>

std::vector<std::vector<double>> combined_calibrate(
	const std::vector<PupilGlint2>& calibData,
	const std::vector<Vec3>& targets,
	const PinholeCameraModel& camera,
	const VecPair3& lights)
{
	// Khởi tạo tham số cần tối ưu
	double kq_left = 600;
	double kq_right = 600;
	double alpha = deg_to_rad(4.65);
	double beta = deg_to_rad(1.5);
	double R = 7.8;
	double K = 4.2;


	// Khởi tạo bài toán tối ưu hóa
	ceres::Problem problem;

	// Thêm residual block cho từng điểm dữ liệu
	for (size_t i = 0; i < calibData.size(); i++) {
		
		auto* cost_function = new ceres::NumericDiffCostFunction<
			CalibrationFunctor, ceres::CENTRAL, 3, 1, 1, 1, 1, 1, 1>(
				new CalibrationFunctor(calibData[i], targets[i], camera, lights, 1.3375, 1));

		problem.AddResidualBlock(cost_function, nullptr, &kq_left, &kq_right, &alpha, &beta, &R, &K);
	}
	
	// Đặt giới hạn cho tham số
	problem.SetParameterLowerBound(&kq_left, 0, 300);
	problem.SetParameterUpperBound(&kq_left, 0, 900);
	problem.SetParameterLowerBound(&kq_right, 0, 300);
	problem.SetParameterUpperBound(&kq_right, 0, 900);

	problem.SetParameterLowerBound(&alpha, 0, deg_to_rad(0));
	problem.SetParameterUpperBound(&alpha, 0, deg_to_rad(5));
	problem.SetParameterLowerBound(&beta, 0, deg_to_rad(0));
	problem.SetParameterUpperBound(&beta, 0, deg_to_rad(5));
	problem.SetParameterLowerBound(&R, 0, 3);
	problem.SetParameterUpperBound(&R, 0, 7);
	problem.SetParameterLowerBound(&K, 0, 3);
	problem.SetParameterUpperBound(&K, 0, 5);
	
	// Cấu hình solver
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 10000;

	// Chạy tối ưu hóa
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << std::endl;

	// Trả về kết quả tối ưu hóa
	std::vector<std::vector<double>> result = {
		{kq_left}, {kq_right}, {alpha}, {beta}, {R}, {K}
	};
	return result;
}
