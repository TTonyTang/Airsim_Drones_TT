#pragma once

#include "FieldMathUtil.hpp"

namespace sxc { namespace field_building {

class ConstantMeshStrategy{

public:

	static void buildMesh(const int strategyId, const Vector2d& pos, const Vector2d& target, vector<Vector2d>& points, pair<int, int>& door) {

		switch (strategyId)
		{
		case 1:
			buildMesh1(points, door);
			break;
		case 2:
			buildMesh2(points, door);
			break;
		default:
			cout << "wrong" << endl;
		}
	}

	static void buildMesh1(vector<Vector2d>& points, pair<int, int>& door){
		const double size = FieldMathUtil::getPerimeter(points) / 100;
		vector<Vector2d> ret;
		FieldMathUtil::homogenizeWithDoor(points, size, ret, door);
		points = ret;
	}

	static void buildMesh2(vector<Vector2d>& points, pair<int, int>& door) {
		const double size = 1.0;
		vector<Vector2d> ret;
		FieldMathUtil::homogenizeWithDoor(points, size, ret, door);
		points = ret;
	}
};

}} //namespace