#pragma once

#include "FieldMathUtil.hpp"
#include "GaussSmoopthen.hpp"

namespace sxc {
	namespace field_building {

		class LinearMeshStrategy {

		public:

			static void buildMesh(const int strategyId, const Vector2d& pos, const Vector2d& target, const double sn, vector<Vector2d>& points, pair<int, int>& door) {
				switch (strategyId)
				{
				case 1:
					break;
				case 2:
					buildMesh2(points, door);
					break;
				default:
					cout << "wrong" << endl;
				}

			}

			static void buildMesh2(vector<Vector2d>& points, pair<int, int>& door) {
				const double size = 1.0;
				vector<Vector2d> ret;
				FieldMathUtil::homogenizeWithDoor(points, size, ret, door);
				points = ret;
			}
		};

	}
} //namespace