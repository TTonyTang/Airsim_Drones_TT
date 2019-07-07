#pragma once

#include "BEMCommon.hpp"

namespace sxc { namespace bem {

class BEMMathUtil{

public:

	template <class F>
	static double unitIntegrate(F f){
		//return gauss<double, 4>::integrate(f, -1, 1);
		return 0;
	}

	static double cross(const Eigen::Vector2d& a, const Vector2d& b) {
		return a.x()*b.y() - b.x()*a.y();
	}

	static bool isPointOnLine(const Vector2d& p1, const Vector2d& p2, const Vector2d& p){
		return sxc::bem::almost_zero(cross(p2 - p1, p1 - p)) &&
			sxc::bem::almost_equal((p1 - p).norm() + (p2 - p).norm(),(p1 - p2).norm());
	}

	/*
	static double distLP(const Vector2d& p1, const Vector2d& p2, const Vector2d& p) {
		return abs(cross(p2 - p1, p1 - p)) / (p2 - p1).norm();
	}

	static double distLPSign(const Vector2d& p1, const Vector2d& p2, const Vector2d& p) {
		return signbit(cross(p1 - p, p2 - p)) * distLP(p1, p2, p);
	}
	

	static double distLPNormalSign(const Vector2d& x, const Vector2d& y, const Vector2d& normal) {
		return distLPSign(y, y + Vector2d(-normal.y(), normal.x()), x);
	}
	*/
	
};

}} //namespace