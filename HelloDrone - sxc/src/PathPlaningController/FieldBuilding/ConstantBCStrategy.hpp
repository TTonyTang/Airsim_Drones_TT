#pragma once

#include "FieldMathUtil.hpp"

namespace sxc { namespace field_building {

class ConstantBCStrategy {

public:

	static void buildCondition(const int strategyId, const Vector2d& pos, const vector<Vector2d>& points, const pair<int, int>& door, map<int, double>& diricheletMap, map<int, double>& neumannMap) {
		vector<Vector2d> nodes;
		FieldMathUtil::getNodes(points, nodes);
		const auto N = int(nodes.size());
		diricheletMap.clear();
		neumannMap.clear();
		switch (strategyId){
			case 1:
				buildCondition1(N, door, diricheletMap, neumannMap);
				break;
			case 2:
				buildCondition2(N, door, diricheletMap, neumannMap);
				break;
			default:
				cout << "wrong" << endl;
		}
	}

	//100£¿ 0£¿Ê²Ã´ÒâË¼
	static void buildCondition1(const int N, const pair<int, int>& door, map<int, double>& diricheletMap, map<int, double>& neumannMap) {
		diricheletMap[0] = 0.0;
		diricheletMap[N - 1] = 0.0;
		for (int i = door.first; i < door.second; i++) {
			diricheletMap[i] = 100;
		}
	}

	static void buildCondition2(const int N, const pair<int, int>& door, map<int, double>& diricheletMap, map<int, double>& neumannMap) {
		diricheletMap[0] = 0.0;
		diricheletMap[N - 1] = 0.0;
		for (int i = door.first; i < door.second; i++) {
			neumannMap[i] = 0;
		}
	}
};

}} //namespace