#include "PersonalCalibration.h"
#include <fstream>
#include <glog/logging.h>

struct DataRow {
	int RegionIndex;
	double PupilX, PupilY, Glint1X, Glint1Y, Glint2X, Glint2Y, GazeX, GazeY, GazeZ;
};

void readFile(const std::string& filename, std::vector<std::vector<PupilGlint2>>& calibData,
	std::vector<Vec3>& targets) {
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Error: Unable to open file '" << filename << "'! Please check if it exists." << std::endl;
		exit(1);
	}

	std::string header;
	std::getline(file, header); // Bỏ qua dòng tiêu đề

	std::vector<DataRow> rows;
	DataRow row;
	while (file >> row.RegionIndex >> row.PupilX >> row.PupilY >> row.Glint1X
		>> row.Glint1Y >> row.Glint2X >> row.Glint2Y >> row.GazeX >> row.GazeY >> row.GazeZ) {
		if (row.RegionIndex < 0) {
			std::cerr << "Warning: Invalid RegionIndex " << row.RegionIndex << " found, skipping." << std::endl;
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
	std::vector<std::vector<DataRow>> grouped(maxIndex + 1);
	for (const auto& r : rows) {
		grouped[r.RegionIndex].push_back(r);
	}

	// Điền calibData và targets, bỏ qua các nhóm rỗng
	calibData.clear();
	targets.clear();
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
		targets.push_back(Vec3(grouped[i][0].GazeX, grouped[i][0].GazeY, grouped[i][0].GazeZ));
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
	std::vector<Vec3> targets;
	readFile("data.txt", calibData, targets);

	VecPair3 lights;
	lights.left = Vec3(-121.038529804592, 70.3381501484736, 18.0648253141379);
	lights.right = Vec3(181.874354550426, -23.7875639906384, -3.03395914937812);

	PinholeCameraModel camera;
	camera.position = Vec3(0, 0, 0);
	camera.principal_point_x = 1014.80750762222;
	camera.principal_point_y = 534.934113781540;
	camera.pixel_size_x = 0.0055;
	camera.pixel_size_y = 0.0055;
	camera.effective_focal_length = sqrt(3043.17471172823*3043.17471172823 +
		3080.82181401356*3080.82181401356);
	const Vec3 screen_center = Vec3(0.359697691901012, -182.304785904820, -124.866720047268);
	Mat3x3 rotation_matrix;
	rotation_matrix << 0.999055371785405, -0.00765182687905128, -0.0427763211618310,
		-0.00765182687905128, 0.938017461597792, -0.346503522757054,
		0.0427763211618310, 0.346503522757054, 0.937072833383196;

	auto results = calibrate(calibData, targets, camera, lights, screen_center, rotation_matrix);

	std::cout << "Global parameters:\n";
	std::cout << "alpha: " << rad_to_deg(results[0][0]) << " deg\n";
	std::cout << "beta: " << rad_to_deg(results[1][0]) << " deg\n";
	std::cout << "R: " << results[2][0] << " mm\n";
	std::cout << "K: " << results[3][0] << " mm\n";
	for (size_t i = 0; i < targets.size(); ++i) {
		std::cout << "Stimulus " << i << ": k_left = " << results[4 + i][0]
			<< ", k_right = " << results[4 + i][1] << "\n";
	}

	return 0;
}