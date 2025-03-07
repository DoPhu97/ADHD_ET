#pragma once
#include "MathTypes.h"
#include <iostream>
class PinholeCameraModel {

	Mat3x3 actual_rotation_matrix;

public:
	// camera intrinsic
	double principal_point_x;
	double principal_point_y;
	double pixel_size_x;
	double pixel_size_y;
	double effective_focal_length;

	// Position in WCS/ translation matrix
	Vec3 position;

	// Rotation matrix in WCS
	Mat3x3 rotation_matrix;

	PinholeCameraModel() {

		position = Vec3(0, 0, 0);
		rotation_matrix = Eigen::Matrix3d::Identity();
		principal_point_x = 0;
		principal_point_y = 0;
		pixel_size_x = 0;
		pixel_size_y = 0;
		effective_focal_length = 0;
	}
	
	/// Transforms the given vector in this camera's image coordinate system to 
	/// the camera coordinate system
	Vec3 ics_to_ccs(const Vec2& pos) const
	{
		return Vec3(
			(pos[0] - principal_point_x) * pixel_size_x,
			(pos[1] - principal_point_y) * pixel_size_y,
			-effective_focal_length * pixel_size_x);
	}

	Vec3 ccs_to_wcs(const Vec3& pos) const
	{
		return rotation_matrix * pos + position;
	}

	Vec3 ics_to_wcs(const Vec2& pos) const
	{
		return ccs_to_wcs(ics_to_ccs(pos));
	}
		
};
