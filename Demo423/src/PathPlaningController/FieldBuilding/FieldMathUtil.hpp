#pragma once

#include "FieldBuildingCommon.hpp"

namespace sxc { namespace field_building {

class FieldMathUtil {

public:
	static void getNodes(const vector<Vector2d>& points, vector<Vector2d>& nodes){
		nodes.clear();
		const auto N = int(points.size());
		for (int i = 0; i < N; i++) {
			nodes.emplace_back((points[i] + points[(i + 1) % N]) / 2);
		}
	}

	static vector<int> farestPoint(const vector<Vector2d>& points, const Vector2d& pos){
		vector<int> index{};
		const auto N = int(points.size());
		double max = 0;
		for (int i = 0; i < N; i++) {
			const double dist = ((points[i] + points[(i + 1) % N]) / 2 - pos).norm();
			if (almost_equal(dist,max)) {
				index.emplace_back(i);
			}else if(dist > max){
				index.clear();
				index.emplace_back(i);
				max = dist;
			}
		}
		return index;
	}

	static vector<int> nearestPoint(const vector<Vector2d>& points, const Vector2d& pos) {
		vector<int> index{};
		const auto N = int(points.size());
		double min = std::numeric_limits<double>::max();
		for (int i = 0; i < N; i++) {
			const double dist = ((points[i] + points[(i + 1) % N]) / 2 - pos).norm();
			if (almost_equal(dist, min)) {
				index.emplace_back(i);
			}else if (dist < min) {
				index.clear();
				index.emplace_back(i);
				min = dist;
			}
		}
		return index;
	}

	static double getPerimeter(const vector<Vector2d>& points){
		double sum = 0;
		const auto N = int(points.size());
		for (int i = 0; i<N; i++) {
			sum += (points[i%N] - points[(i + 1) % N]).norm();
		}
		return sum;
	}

	static void homogenizeWithDoor(const vector<Vector2d>& points, const double size, vector<Vector2d>& ret, pair<int, int>& door){
		const auto N = int(points.size());
		for (int i = 0; i<N; i++) {
			const auto seg = splitLineSegment(points[i%N], points[(i + 1) % N], size);
			if(i==1){
				door.first = int(ret.size()+seg.size());
			}else if(i==N-2){
				door.second = int(ret.size())-1;
			}
			ret.insert(ret.end(), seg.begin(), seg.end());
		}
	}

	static void homogenize(const vector<Vector2d>& points, const double size, vector<Vector2d>& ret) {
		ret.clear();
		const auto N = int(points.size());
		for (int i = 0; i<N; i++) {
			const auto seg = splitLineSegment(points[i%N], points[(i + 1) % N], size);
			ret.insert(ret.end(), seg.begin(), seg.end());
		}
	}

	static void meanSample(const vector<Vector2d>& points, const int totalNumber, vector<Vector2d>& ret) {
		ret.clear();
		const auto NN = int(points.size());
		if(NN<=totalNumber){
			ret = points;
		} else {
			const int gap = NN / totalNumber;
			for (int i = 0; i < totalNumber; i++) {
				Vector2d p = Vector2d::Zero();
				for (int j = i * gap; j < (i + 1)* gap; j++) {
					p += points[j];
				}
				p /= gap;
				ret.emplace_back(p);
			}
		}
	}

	static vector<Vector2d> splitLineSegment(const Vector2d& start, const Vector2d& end, const double size) {
		const Vector2d delta = end - start;
		const auto N = delta.norm()>size?int(delta.norm()/size):1;
		vector<Vector2d> points(N);
		for (int i = 0; i < N; i++) {
			points[i] = start + delta * double(i) / double(N);
		}
		return points;
	}

	static vector<pair<int, int>> getBlankSegment(const vector<Vector2d>& viewPoints, const Vector2d& pos, const int start, const int end, const double maxdepth, const double sn) {
		const auto N = int(viewPoints.size());
		vector<pair<int, int>> segment;
		int index = -1;
		const double width = 2 * sn * maxdepth;
		for (int i = start; (i%N) != end; i++) {
			if (abs((viewPoints[i%N]-pos).norm() - maxdepth)< width && index == -1) {
				index = (i-1) % N;
			}
			if (abs((viewPoints[i%N] - pos).norm() - maxdepth)>= width && index != -1) {
				segment.emplace_back(pair<int, int>{index, i % N});
				index = -1;
			}
		}
		if (index != -1) {
			segment.emplace_back(pair<int, int>{index, end});
		}
		return segment;
	}

	static double getAngle(const Vector2d& x, const Vector2d& y, const Vector2d& target) {
		auto l1 = target - x;
		auto l2 = target - y;
		return atan2(l1.y(), l1.x()) - atan2(l2.y(), l2.x());
	}

	static double regularAngle(const double ang) {
		if (ang < 0) {
			return regularAngle(ang + 2 * M_PI);
		}
		else if (ang > 2 * M_PI) {
			return regularAngle(ang - 2 * M_PI);
		}
		else {
			return ang;
		}
	}
};

}} //namespace