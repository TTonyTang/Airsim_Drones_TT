#pragma once

#include<vector>
#include "Eigen/Eigen"
//NÊÇÊ²Ã´£¿
namespace sxc { namespace field_building {
using std::vector;
using std::pair;
using std::make_pair;

using Eigen::Vector2d;
class ClosedBoundary {

public:
	int N = 0;

	vector<Vector2d> points;

	vector<double> dists;

	vector<bool> masks;

	Vector2d pos;

	double radius;

	double limit;

	//double safeGap;

	ClosedBoundary(const vector<Vector2d> points, const Vector2d pos, const double radius, const double limit) :points(points), pos(pos), radius(radius), limit(limit){
		for (const auto& p : points) {
			const auto d = (p - pos).norm();
			dists.emplace_back(d);
			masks.emplace_back(abs(radius - d) < limit);
		}
		N = int(points.size());
	}

	void erase(const int i, const int j){
		const int regI = getRegularIndex(i);
		const int regJ = getRegularIndex(j+1);
		if(regI<=regJ){
			points.erase(points.begin() + regI, points.begin() + regJ);
			dists.erase(dists.begin() + regI, dists.begin() + regJ);
			masks.erase(masks.begin() + regI, masks.begin() + regJ);
		}else{
			points.erase(points.begin() + regI, points.end());
			dists.erase(dists.begin() + regI, dists.end());
			masks.erase(masks.begin() + regI, masks.end());

			points.erase(points.begin(), points.begin() + regJ);
			dists.erase(dists.begin(), dists.begin() + regJ);
			masks.erase(masks.begin(), masks.begin() + regJ);
		}
		N = int(points.size());
	}

	Vector2d getPoint(const int i) const {
		return points[((i%N) + N) % N];
	}

	Vector2d getPointExtended(const int i) const{
		return pos + radius * (getPoint(i) - pos).normalized();
	}

	double getDist(const int i) const {
		return dists[((i%N) + N) % N];
	}

	bool isOpen(const int i) const {
		return masks[((i%N) + N) % N];
	}

	int getRegularIndex(const int i) const {
		return ((i%N) + N) % N;
	}

	vector<Vector2d> getPoints() const {
		return points;
	}

	int nextOpen(const int i) const {
		if(isOpen(i+1)){
			return getRegularIndex(i+1);
		} else {
			return nextOpen(i + 1);
		}
	}

	int nextClose(const int i) const {
		if (!isOpen(i + 1)) {
			return getRegularIndex(i + 1);
		} else {
			return nextClose(i + 1);
		}
	}

	vector<pair<int, int>> getGapSegment(const int start, const int end) const {
		vector<pair<int, int>> segments;
		int index = -1;
		const int length = getRegularIndex(end - start);
		for (int i = start; i <= start + length; i++) {
			//cout << radius << ":" << getDist(i) << ":" << isOpen(i) << ":" << endl;
			const bool open = isOpen(i);
			if (open && index == -1) {
				index = getRegularIndex(i - 1);
			}
			if (!open && index != -1) {
				//if ((getPointExtended(i) - getPointExtended(index)).norm() > safeGap) {
					segments.emplace_back(make_pair(index, getRegularIndex(i)));
				//}
				/*
				else
				{
					cout << "---a---" << endl;
					cout << getPoint(i) << endl;
					cout << getPoint(index) << endl;
				}
				*/
				index = -1;
			}
		}
		if (index != -1) {
			//if ((getPointExtended(end) - getPointExtended(index)).norm() > safeGap) {
				segments.emplace_back(make_pair(index, getRegularIndex(end)));
			//}
			/*
			else
			{
				cout << "---b---" << endl;
				cout << getPoint(end) << endl;
				cout << getPoint(index) << endl;
			}
			*/
		}
		if(int(segments.size())==0){
			cout << "Not Feasible!" << endl;
		}
		/*
		for(const auto& seg:segments){
			cout << seg.first << "::" << seg.second << endl;
		}
		cout << segments.size() << endl;
		*/
		return segments;
	}

	vector<pair<int, int>> getAllGapSegment() const {
		vector<pair<int, int>> segments;
		int start = 0;
		int end = N - 1;
		for (int i = 0; i < N; i++) {
			if (!isOpen(i - 1) && isOpen(i)) {
				start = i;
				end = i - 1 + N;
				break;
			}
		}
		return getGapSegment(start, end);
	}

	vector<pair<int, int>> getRangeGapSegment(const Vector2d& target, const double startAngle, const double endAngle) const {
		const auto ret = angleRangeToIndexRange(startAngle, endAngle);
		//cout << startAngle << "," << endAngle << endl;
		//cout << ret.first << "," << ret.second << endl;
		return getGapSegment(ret.first, ret.second);
	}

	bool isforward(const int i, const Vector2d& ref) const {
		const auto& vec = getPoint(i)-pos;
		return ref.x()*vec.y() - vec.x()*ref.y() > 0;
	}

	pair<int, int> angleRangeToIndexRange(const double startAngle, const double endAngle) const {
		vector<pair<int, int>> segments;
		const Vector2d startVector(cos(startAngle), sin(startAngle));
		const Vector2d endVector(cos(endAngle), sin(endAngle));
		int index = -1;
		int start = 0;
		int end = N - 1;
		for (int i = 0; i < N; i++) {
			const bool inside = isforward(i, startVector) && !isforward(i, endVector);
			const bool preInside = isforward(i-1, startVector) && !isforward(i-1, endVector);
			if (!preInside && inside) {
				start = i;
				end = i - 1 + N;
				break;
			}
		}
		//cout << "start:"<< start << ", end:" << end << endl;
		for (int i=start; i <= end; i++) {
			const bool inside = isforward(i, startVector) && !isforward(i, endVector);
			if (inside && index == -1) {
				index = getRegularIndex(i);
			}
			if (!inside && index != -1) {
				segments.emplace_back(make_pair(index, getRegularIndex(i-1)));
				index = -1;
			}
		}
		if (index != -1) {
			segments.emplace_back(make_pair(index, getRegularIndex(end)));
		}
		/*
		for(const auto& seg:segments)
		{
			cout << seg.first << "::" << seg.second << endl;
		}
		*/
		double max = 0;
		int mainIndex = 0;;
		for (int i = 0; i < int(segments.size()); i++) {
			const auto& s = segments[i];
			const double length = (getPointExtended(s.second) - getPointExtended(s.first)).norm();
			max = length > max ? length:max;
			mainIndex = i;
		}
		return segments[mainIndex];
	}

	void connectTarget(const Vector2d& target, const pair<int, int> door, vector<Vector2d>& retPoints) const {
		retPoints.clear();
		retPoints.emplace_back(target);
		const int xIndex = door.first;
		const int yIndex = door.second;
		const auto& px = getPoint(xIndex);
		const auto& py = getPoint(yIndex);
		const auto extendPx = getPointExtended(xIndex);
		const auto extendPy = getPointExtended(yIndex);
		if (!extendPy.isApprox(py)) {
			retPoints.emplace_back(extendPy);
		}
		const int length = getRegularIndex(xIndex - yIndex);
		for (int i = yIndex; i <= yIndex + length; i++) {
			retPoints.emplace_back(getPoint(i));
		}
		if (!extendPx.isApprox(px)) {
			retPoints.emplace_back(extendPx);
		}
	}

	void connectTarget(const Vector2d& target, const pair<int, int> door, const pair<Vector2d, Vector2d> ps, vector<Vector2d>& retPoints) const {
		retPoints.clear();
		retPoints.emplace_back(target);
		const int xIndex = door.first;
		const int yIndex = door.second;
		const auto& px = ps.first;
		const auto& py = ps.second;
		retPoints.emplace_back(py);
		const int length = getRegularIndex(xIndex - yIndex);
		for (int i = yIndex+1; i < yIndex + length; i++) {
			retPoints.emplace_back(getPoint(i));
		}
		retPoints.emplace_back(px);
	}
};
}} //namespace