#include "PinholeCameraModel.h"

PinholeCameraModel::PinholeCameraModel() {
	position = Vec3(0, 0, 0);
	principal_point_x = 1;
	principal_point_y = 1;
	pixel_size_x = 1;
	pixel_size_y = 1;
	effective_focal_length = 1;
}

Vec3 PinholeCameraModel::ics_to_ccs(const Vec2& pos) const {
	return Vec3(
		(pos[0] - principal_point_x) * pixel_size_x,
		(pos[1] - principal_point_y) * pixel_size_y,
		-effective_focal_length * pixel_size_x);
}

Vec3 PinholeCameraModel::ccs_to_wcs(const Vec3& pos) const {
	return pos + position;
}

Vec3 PinholeCameraModel::ics_to_wcs(const Vec2& pos) const {
	return ccs_to_wcs(ics_to_ccs(pos));
}