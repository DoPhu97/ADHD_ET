#pragma once
#include "MathTypes.h"

class PinholeCameraModel {
public:
	double principal_point_x;
	double principal_point_y;
	double pixel_size_x;
	double pixel_size_y;
	double effective_focal_length;
	Vec3 position;

	PinholeCameraModel();
	Vec3 ics_to_ccs(const Vec2& pos) const;
	Vec3 ccs_to_wcs(const Vec3& pos) const;
	Vec3 ics_to_wcs(const Vec2& pos) const;
};