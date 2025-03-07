#include "PersonalCalibration.h"
#include <ceres/ceres.h>
#include <iostream>

std::vector<std::vector<double>> calibrate(
	const std::vector<PupilGlint2>& calibData,
	const std::vector<Vec3>& targets,
	const PinholeCameraModel& camera,
	const VecPair3& lights,
	const Vec3 screen_center,
	const Mat3x3 rotation_matrix)
{

	double alpha = deg_to_rad(4);
	double beta = deg_to_rad(2);
	double R = 7.8;
	double K = 4.2;

	ceres::Problem problem;
	std::cout << calibData.size();

	
	for (int i = 0; i < calibData.size(); i++) {
		
		auto* cost_function = new ceres::NumericDiffCostFunction
			<PersonalCalibrationFunctor, ceres::CENTRAL, 3, 1, 1, 1, 1>
			(new PersonalCalibrationFunctor(calibData[i], 
				targets[i], 
				camera, lights, 
				1.3375, 1,
				screen_center, rotation_matrix));

		problem.AddResidualBlock(cost_function, nullptr, &alpha, &beta, &R, &K);
	}
	

	problem.SetParameterLowerBound(&alpha, 0, deg_to_rad(1));
	problem.SetParameterUpperBound(&alpha, 0, deg_to_rad(5));
	problem.SetParameterLowerBound(&beta, 0, deg_to_rad(1));
	problem.SetParameterUpperBound(&beta, 0, deg_to_rad(5));
	problem.SetParameterLowerBound(&R, 0, 7);
	problem.SetParameterUpperBound(&R, 0, 8);
	problem.SetParameterLowerBound(&K, 0, 4);
	problem.SetParameterUpperBound(&K, 0, 5);
	
	
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 10000;

	
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << std::endl;

	
	std::vector<std::vector<double>> result = {
		{alpha}, {beta}, {R}, {K}
	};
	return result;
}

Vec3 calculate_cornea_center_wcs(
	const PupilGlint2 pupilGlint2,
	const VecPair3 lights,
	const PinholeCameraModel camera,
	double R)
{

	double kq_left = 600;
	double kq_right = 600;

	ceres::Problem problem;
	auto* cost_function = new ceres::NumericDiffCostFunction
		<CorneaDistanceFunctor, ceres::CENTRAL, 3, 1, 1>
		(new CorneaDistanceFunctor(pupilGlint2, camera, lights, R));

	problem.AddResidualBlock(cost_function, nullptr, &kq_left, &kq_right);


	problem.SetParameterLowerBound(&kq_left, 0, 300);
	problem.SetParameterUpperBound(&kq_left, 0, 900);
	problem.SetParameterLowerBound(&kq_right, 0, 300);
	problem.SetParameterUpperBound(&kq_right, 0, 900);

	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.linear_solver_type = ceres::DENSE_QR;
	//	options.function_tolerance = 1e-8;
	//	options.gradient_tolerance = 1e-12;
	options.max_num_iterations = 10000;
	//	options.min_line_search_step_size = 1e-3;
	//	options.use_nonmonotonic_steps = true;

	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);

	Vec3 q1 = calculate_q(kq_left, camera.position,camera.ics_to_wcs(pupilGlint2.glints.left));
	Vec3 q2 = calculate_q(kq_right, camera.position,camera.ics_to_wcs(pupilGlint2.glints.right));

	Vec3 c1 = calculate_cornea_center(q1, lights.left, camera.position, R);
	Vec3 c2 = calculate_cornea_center(q2, lights.right, camera.position, R);

	// Trả về trung bình của hai trung tâm giác mạc
	return (c1 + c2) * 0.5;

}


