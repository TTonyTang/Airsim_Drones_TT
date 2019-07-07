#pragma once

#include "FieldMathUtil.hpp"
#include "ClosedBoundary.hpp"

namespace sxc { namespace field_building {

class BoundaryStrategy {

public:

	static void buildBoundary(const int strategyId, ClosedBoundary& boundary, const double maxDist, const Vector2d& target, vector<Vector2d>& points, Vector2d& tempTarget) {
		switch (strategyId)
		{
		case 1:
			buildBoundary1(boundary, target, maxDist, points, tempTarget);
			break;
		case 2:
			buildBoundary2(boundary, target, maxDist, points, tempTarget);
			break;
		case 3:
			buildBoundary3(boundary, target, maxDist, points, tempTarget);
			break;
		case 4:
			buildBoundary4(boundary, target, maxDist, points, tempTarget);
			break;
		default:
			cout << "wrong" << endl;
		}
	}

private:

	static void buildBoundary1(ClosedBoundary& boundary, const Vector2d& target, const double maxDist, vector<Vector2d>& points, Vector2d& tempTarget) {
		//const auto N = int(viewPoints.size());
		scaleTarget(boundary.pos, target, maxDist, tempTarget);
		vector<pair<int, int>> blankSegments;
		getFeasibleGapSegment(boundary, tempTarget, blankSegments);
		pair<int, int> door;
		double maxgap = 0;
		for (const auto& blankSegment : blankSegments) {
			const auto& x = boundary.getPointExtended(blankSegment.first);
			const auto& y = boundary.getPointExtended(blankSegment.second);
			const double gap = (x - y).norm();
			if (gap>maxgap) {
				door = blankSegment;
				maxgap = gap;
			}
		}
		boundary.connectTarget(tempTarget, door, points);
	}

	static void buildBoundary2(ClosedBoundary& boundary, const Vector2d& target, const double maxDist, vector<Vector2d>& points, Vector2d& tempTarget) {
		//const auto N = int(viewPoints.size());
		scaleTarget(boundary.pos,target, maxDist, tempTarget);
		vector<pair<int, int>> blankSegments;
		getFeasibleGapSegment(boundary, tempTarget, blankSegments);
		pair<int, int> door;

		double maxcos = -1;
		for (const auto& blankSegment : blankSegments) {
			//cout << blankSegment.first << "::" << blankSegment.second << endl;
			const auto& px = boundary.getPointExtended(blankSegment.first);
			const auto& py = boundary.getPointExtended(blankSegment.second);
			const auto node = (px + py) / 2;
			if ((node - boundary.pos).norm()<boundary.limit) {
				door = blankSegment;
				//maxcos = 1;
				break;
			}
			const auto cos = (node - boundary.pos).normalized().dot((tempTarget - boundary.pos).normalized());
			if (cos > maxcos) {
				door = blankSegment;
				maxcos = cos;
			}
		}
		//cout << door.first<<":"<<door.second << endl;
		boundary.connectTarget(tempTarget, door, points);
	}

	static void buildBoundary3(ClosedBoundary& boundary, const Vector2d& target, const double maxDist, vector<Vector2d>& points, Vector2d& tempTarget) {
		scaleTarget(boundary.pos, target, maxDist, tempTarget);
		vector<pair<int, int>> blankSegments;
		getFeasibleGapSegment(boundary, tempTarget, blankSegments);
		pair<int, int> door;

		double maxcos = -1;
		for (const auto& blankSegment : blankSegments) {
			//cout << blankSegment.first << "::" << blankSegment.second << endl;
			const auto& px = boundary.getPointExtended(blankSegment.first);
			const auto& py = boundary.getPointExtended(blankSegment.second);
			const auto node = (px + py) / 2;
			if ((node - boundary.pos).norm()<boundary.limit) {
				door = blankSegment;
				//maxcos = 1;
				break;
			}
			const auto cosx = (px - boundary.pos).normalized().dot((tempTarget - boundary.pos).normalized());
			const auto cosy = (py - boundary.pos).normalized().dot((tempTarget - boundary.pos).normalized());
			const auto cos = std::max(cosx, cosy);
			if (cos > maxcos) {
				door = blankSegment;
				maxcos = cos;
			}
		}
		//cout << door.first<<":"<<door.second << endl;
		boundary.connectTarget(tempTarget, door, points);
	}

	static double cross(const Eigen::Vector2d& a, const Vector2d& b) {
		return a.x()*b.y() - b.x()*a.y();
	}

	static void buildBoundary4(ClosedBoundary& boundary, const Vector2d& target, const double maxDist, vector<Vector2d>& points, Vector2d& tempTarget) {
				scaleTarget(boundary.pos,target, maxDist, tempTarget);
		vector<pair<int, int>> blankSegments;
		getFeasibleGapSegment(boundary, tempTarget, blankSegments);
		pair<int, int> door;
		double maxgap = 0;
		for (const auto& blankSegment : blankSegments) {
			//cout << blankSegment.first << "::" << blankSegment.second << endl;
			const auto& x = boundary.getPoint(blankSegment.first);
			const auto& y = boundary.getPoint(blankSegment.second);
			const double gap = (x - y).norm();
			if (gap>maxgap) {
				door = blankSegment;
				maxgap = gap;
			}
		}
		boundary.connectTarget(tempTarget, door, points);
	}


	static void scaleTarget(const Vector2d& pos, const Vector2d& target, const double maxDist, Vector2d& tempTarget){
		const auto dist = (target - pos).norm();
		tempTarget = target;
		if (dist>maxDist) {
			tempTarget = pos + (maxDist)*((target - pos).normalized());
		}
	}

	static void getFeasibleGapSegment(ClosedBoundary& boundary, const Vector2d& tempTarget, vector<pair<int, int>>& blankSegments) {
		auto targetDirection = tempTarget - boundary.pos;
		const auto targetTheta = atan2(targetDirection.y(), targetDirection.x());

		double alpha;
		if (targetDirection.norm() - boundary.radius>boundary.limit) {
			alpha = abs(acos(boundary.radius / targetDirection.norm()));
		} else {
			alpha = M_PI / 2;
		}
		//cout << targetTheta << endl;
		//cout << alpha << endl;
		const double end = FieldMathUtil::regularAngle(targetTheta + alpha);
		const double start = FieldMathUtil::regularAngle(targetTheta - alpha);
		blankSegments = boundary.getRangeGapSegment(tempTarget, start, end);
	}

};

}} //namespace