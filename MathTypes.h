#pragma once
#include "Eigen/Core"


typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix3d Mat3x3;

struct VecPair3 {
	Vec3 left;
	Vec3 right;
};

struct VecPair2 {
	Vec2 left;
	Vec2 right;
};

inline Vec3 normalized(const Vec3& a)
{
	return a.normalized();
}

inline double dot(const Vec3& a, const Vec3& b)
{
	return a.dot(b);
}

inline double deg_to_rad(double a)
{
	return a * 3.141592653589793 / 180.;
}

inline double rad_to_deg(double a)
{
	return a * 180./ 3.141592653589793;
}

inline double squared_length(const Vec3& a)
{
	return a.squaredNorm();
}

inline Mat3x3 mat_prod(const Mat3x3& a, const Mat3x3& b)
{
	return a * b;
}


inline Vec3 mat3vec3_prod(const Mat3x3& a, const Vec3& b)
{
	return a * b;
}