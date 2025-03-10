// Gaze Estimation for one camera and two light sources
// All equations are from 
// Remote, Non - Contact Gaze Estimation with Minimal Subject Cooperation
// Guestrin, Elias Daniel
// https://tspace.library.utoronto.ca/handle/1807/24349

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "MathTypes.h"
#include "PinholeCameraModel.h"
#include "OneCameraSpherical.h"
#include "PersonalCalibration.h"

struct DataRow {
	int RegionIndex;
	double PupilX, PupilY, Glint1X, Glint1Y, Glint2X, Glint2Y, GazeX, GazeY, GazeZ;
};

void readFile(const std::string &filename, std::vector<DataRow> &data) {

	std::ifstream file(filename); // Open the file
	if (!file.is_open()) {
		std::cerr << "Error: Unable to open file!" << std::endl;
	}
	std::string header;
	std::getline(file, header); // Remove the first line (header)
	DataRow row;

	while (file >> row.RegionIndex >> row.PupilX >> row.PupilY >> row.Glint1X 
		>> row.Glint1Y >> row.Glint2X >> row.Glint2Y >> row.GazeX >> row.GazeY >> row.GazeZ) 
	{
		data.push_back(row);
	}

	file.close();
}

 int main(int argc, char** argv)
{
	 google::InitGoogleLogging(argv[0]);
	 
	 std::vector<DataRow> data;
	 readFile("data.txt", data);
	 VecPair3 lights;//cm
	 lights.left = Vec3(-121.038529804592, 70.3381501484736, 18.0648253141379);
	 lights.right = Vec3(181.874354550426, -23.7875639906384, -3.03395914937812);

	// WCS has its origin at the camera 1 position
	PinholeCameraModel camera;
	camera.position = Vec3(0, 0, 0);
	camera.principal_point_x = 1014.80750762222;
	camera.principal_point_y = 534.934113781540;
	camera.pixel_size_x = 0.0055;//mm
	camera.pixel_size_y = 0.0055;
	camera.effective_focal_length = sqrt(3080.82181401356*3080.82181401356 +
		3043.17471172823*3043.17471172823);
	
	//Scene parameters to get poi in pixels
	//const double display_surface_size_x = ;
	//const double display_surface_size_y = ;
	const double screen_resolution_x = 0.27;// mm/pixel
	const double screen_resolution_y = 0.27;
	const double screen_pixel_size_x = 1920;
	const double screen_pixel_size_y = 1200;
	const Vec3 screen_center = Vec3(0.359697691901012, -182.304785904820, -124.866720047268);


	Mat3x3 rotation_matrix;
	rotation_matrix << 0.999055371785405, -0.00765182687905128, -0.0427763211618310,
		-0.00765182687905128, 0.938017461597792, -0.346503522757054,
		0.0427763211618310, 0.346503522757054, 0.937072833383196;
	
	std::vector<Vec3> targets;
	std::vector<PupilGlint2> calibData;
	PupilGlint2 pupiGlint;


	for (int i = 0; i < data.size(); i++) {
		
		targets.push_back(Vec3(data[i].GazeX, data[i].GazeY, data[i].GazeZ));
		pupiGlint.pupil = Vec2(data[i].PupilX, data[i].PupilY);
		pupiGlint.glints.left = Vec2(data[i].Glint1X, data[i].Glint1Y);
		pupiGlint.glints.right = Vec2(data[i].Glint2X, data[i].Glint2Y);
		calibData.push_back(pupiGlint);
	}
	
	/*
	for (int i = 0; i < data.size(); i++) {
		std::cout << targets[i][0]<<" "<< targets[i][1]<<" "<< targets[i][2] << "\n";
	}
	*/
	std::vector<std::vector<double>> results = calibrate(
		calibData, targets, camera, lights,	screen_center, rotation_matrix);
	
	
	std::cout << "Calibration results:\n";
	std::cout << "alpha: " << rad_to_deg(results[0][0]) << "\n";
	std::cout << "beta: " << rad_to_deg(results[1][0]) << "\n";
	std::cout << "R: " << results[2][0] << "\n";
	std::cout << "K: " << results[3][0] << "\n";
	
	return 0;
}

