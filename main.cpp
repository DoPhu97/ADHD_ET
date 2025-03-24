//#include "PersonalCalibration.h"
#include "R_estimation.h"
#include <fstream>
#include <glog/logging.h>
#include <iostream>


struct DataRow2CalR {
	int RegionIndex;
	double PupilX, PupilY, Glint1X, Glint1Y, Glint2X, Glint2Y, CorneaX, CorneaY, CorneaZ;
};

void readFile2CalR(const std::string& filename, std::vector<std::vector<PupilGlint2>>& calibData,
	std::vector<Vec3>& cornea_truth) 
{
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Error: Unable to open file '" << filename << "'! Please check if it exists.\n";
		exit(1);
	}

	std::string header;
	std::getline(file, header); // Bỏ qua dòng tiêu đề

	std::vector<DataRow2CalR> rows;
	DataRow2CalR row;
	while (file >> row.RegionIndex >> row.PupilX >> row.PupilY >> row.Glint1X
		>> row.Glint1Y >> row.Glint2X >> row.Glint2Y >> row.CorneaX >> row.CorneaY >> row.CorneaZ) {
		if (row.RegionIndex < 0) {
			std::cerr << "Warning: Invalid RegionIndex " << row.RegionIndex << " found, skipping.\n";
			continue;
		}
		rows.push_back(row);
	}

	if (rows.empty()) {
		std::cerr << "Error: No valid data found in '" << filename << "'!" << std::endl;
		file.close();
		exit(1);
	}

	// Tìm giá trị RegionIndex lớn nhất
	int maxIndex = -1;
	for (const auto& r : rows) {
		maxIndex = std::max(maxIndex, r.RegionIndex);
	}

	// Nhóm dữ liệu theo RegionIndex
	std::vector<std::vector<DataRow2CalR>> grouped(maxIndex + 1);
	for (const auto& r : rows) {
		grouped[r.RegionIndex].push_back(r);
	}

	// Điền calibData và targets, bỏ qua các nhóm rỗng
	calibData.clear();
	cornea_truth.clear();
	for (int i = 0; i <= maxIndex; ++i) {
		if (grouped[i].empty()) {
			continue; // Bỏ qua nhóm rỗng
		}
		std::vector<PupilGlint2> groupData;
		for (const auto& r : grouped[i]) {
			PupilGlint2 pg;
			pg.pupil = Vec2(r.PupilX, r.PupilY);
			pg.glints.left = Vec2(r.Glint1X, r.Glint1Y);
			pg.glints.right = Vec2(r.Glint2X, r.Glint2Y);
			groupData.push_back(pg);
		}
		calibData.push_back(groupData);
		cornea_truth.push_back(Vec3(grouped[i][0].CorneaX, grouped[i][0].CorneaY, grouped[i][0].CorneaZ));
	}

	if (calibData.empty()) {
		std::cerr << "Error: No valid groups found after processing '" << filename << "'!" << std::endl;
		file.close();
		exit(1);
	}

	file.close();
	std::cout << "Read " << calibData.size() << " groups from '" << filename << "'." << std::endl;
}

int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	std::vector<std::vector<PupilGlint2>> calibData;
	std::vector<Vec3> cornea_truth;
	readFile2CalR("data2calR.txt", calibData, cornea_truth);

	VecPair3 lights;
	lights.left = Vec3(-121.038529804592, 70.3381501484736, 18.0648253141379);
	lights.right = Vec3(181.874354550426, -23.7875639906384, -3.03395914937812);

	PinholeCameraModel camera;
	camera.position = Vec3(0, 0, 0);
	camera.principal_point_x = 1014.80750762222;
	camera.principal_point_y = 534.934113781540;
	camera.pixel_size_x = 0.0055;
	camera.pixel_size_y = 0.0055;
	camera.effective_focal_length = sqrt(3043.17471172823 * 3043.17471172823 +
		3080.82181401356 * 3080.82181401356);

	// Giai đoạn 1: Tìm k_left và k_right với R là hằng số
	const double initial_R = 7.8;  // Giá trị R ban đầu cố định
	auto k_values = calculate_k_values(calibData, camera, lights, cornea_truth, initial_R);
	if (k_values.empty()) {
		std::cerr << "Phase 1 failed!" << std::endl;
		return 1;
	}

	// In kết quả k_values
	for (int i = 0; i < k_values.size(); ++i) {
		std::cout << "Point " << i << ": k_left = " << k_values[i][0]
			<< ", k_right = " << k_values[i][1] << "\n";
	}

	// Giai đoạn 2: Tìm R với k_left và k_right đã biết
	double final_R = calculate_final_R(calibData, camera, lights, cornea_truth, k_values);
	std::cout << "Final R: " << final_R << " mm\n";

	return 0;
}