#pragma once

#include "Common.hpp"
#include "Firmware.hpp"
//#include "FieldBuilding/BoundaryDetection.hpp"
#include "BEM/BEM2DConstant.hpp"
#include "FieldBuilding/ConstantMeshStrategy.hpp"
#include "FieldBuilding/ConstantBCStrategy.hpp"
#include "FieldBuilding/BoundaryStrategy.hpp"
#include "BEM/BEM2DLinear.hpp"
//#include "FieldBuilding/GaussSmoopthen.hpp"
#include "SGSmooth.hpp"
#include "GaussSmoopthen.hpp"
#include "FieldBuilding/ClosedBoundary.hpp"

namespace sxc { namespace path_planning_controller {

//using field_building::BoundaryDetection;
using BoundaryStrategy = field_building::BoundaryStrategy;

using BEM = bem::BEM2DConstantAnalytical;
using MeshStrategy = field_building::ConstantMeshStrategy;
using BCStrategy = field_building::ConstantBCStrategy;

//using BEM = bem::BEM2DLinear;

using std::mutex;
using std::tuple;
using std::vector;
using std::map;
using std::ofstream;
using std::string;
using std::thread;
using Eigen::Vector2d;

class FieldBuilding{
	
public:

	FieldBuilding(Firmware* firmware) : firmware(firmware) {}
	~FieldBuilding() = default;

	void start(const Vector2d& target, const double speed) {
		cout << "before field_building" << endl;
		flag = true;
		target_ = target;
		speed_ = speed;
		thd = thread(&FieldBuilding::run, this);
		cout << "field_building start ready" << endl;
	}

	void stop() {
		flag = false;
		if (thd.joinable()) {
			thd.join();
		}
	}

	void run() {
		cout << "field_building runing" << endl;
		while (flag) {
			//cout << "building" << endl;
			period();
		}
	}

	//??Flux
	Vector2d getFlux(const Vector2d& x) {
		Vector2d ret = Vector2d::Zero();
		mtx.lock();
		if(bemcase!=nullptr) {
			ret = bemcase->getFlux(x);
		}
		mtx.unlock();
		return ret.normalized();
	}

private:

	Firmware* firmware;
	BEM* bemcase = nullptr;

	thread thd;
	mutex mtx{};
	bool flag = false;

	Vector2d target_ = Vector2d::Zero();
	double speed_ = 1.0;

	int num = 0;

	static void cutViewPoints(const Vector2d& pos, const double maxdepth, vector<Vector2d>& viewPoints){
		for(auto& p: viewPoints){
			const auto d = (p - pos).norm();
			if(d>maxdepth){
				p = pos+(p - pos).normalized()*maxdepth;
			}
		}
	}

	static void smoothing(const vector<Vector2d>& ps, vector<Vector2d>& viewPoints){
		const auto N = int(ps.size());

		vector<Vector2d> ret;
		const double size = 0.1;
		field_building::FieldMathUtil::homogenize(ps, size, ret);

		const auto NN = int(ret.size());
		vector<double> pxs;
		vector<double> pys;
		for (const auto& p : ret) {
			pxs.emplace_back(p.x());
			pys.emplace_back(p.y());
		}

		//cout << "sizea" << dists.size() << endl;
		//dists = gaussSmoothen(dists, firmware->sn*firmware->maxDepth, 5);
		pxs = sg_smooth(pxs, 9, 1);
		pys = sg_smooth(pys, 9, 1);

		vector<Vector2d> tempPoints;
		for (int i = 0; i < NN; i++) {
			tempPoints.emplace_back(Vector2d(pxs[i], pys[i]));
		}

		field_building::FieldMathUtil::meanSample(tempPoints, 100, viewPoints);
	}

	void period() {
		tic();
		
		auto* temp = new BEM();
		//const int N = 60;

		Vector2d pos;
		//double yaw;
		vector<Vector2d> ps;

		firmware->getBoundary(pos, ps);


		const auto dist = (target_ - pos).norm();
		double radius = firmware->maxDepth;
		const double limit = 3 * firmware->sn*firmware->maxDepth;
		//const double safeGap = speed_ * firmware->minGap;

		if(dist<firmware->positionTolerance){
			return;
		}

		field_building::ClosedBoundary ori(ps, pos, radius, limit);

		/*
		vector<std::pair<int, int>> segs = ori.getAllGapSegment();
		for(const auto& seg: segs){
			if(seg.first!= seg.second && (ori.getPoint(seg.first)-ori.getPoint(seg.second)).norm()<firmware->minGap){
				cout << seg.first << "," << seg.second;
				ori.erase(seg.first+1, seg.second-1);
			}
		}
		*/

		/*
		if (dist<firmware->maxDepth) {
			radius = dist;
		}
		*/

		vector<Vector2d> viewPoints;
		smoothing(ori.getPoints(),viewPoints);
		//viewPoints = ps;
		cutViewPoints(pos, radius, viewPoints);

		field_building::ClosedBoundary boundary(viewPoints, pos, radius, limit);

		//BoundaryDetection::boundaryDetection(pos, maxDepth, N, viewPoints);

		vector<Vector2d> points;
		Vector2d tempTarget;
		BoundaryStrategy::buildBoundary(firmware->boundaryStrategy, boundary, firmware->maxDist,target_, points, tempTarget);

		cout << tempTarget << endl;
		std::pair<int, int> door;

		MeshStrategy::buildMesh(firmware->meshStrategy, pos, tempTarget, points, door);

		temp->setMesh(points);

		map<int, double> bcs1,bcs2;
		BCStrategy::buildCondition(firmware->bcStrategy, pos, points, door, bcs1,bcs2);
		temp->setBoundaryCondition(bcs1, bcs2);

		temp->solve();

		mtx.lock();
		delete bemcase;
		bemcase = temp;
		mtx.unlock();

		cout << "build successfully: No." << ++num << endl;
		cout << "number of points: " << points.size() << endl;
		//cout << "speed:"<<speed_ << endl;
		//cout << "pos:"<<pos << endl;
		//cout << "flux:"<<bemcase->getFlux(pos) << endl;
		
		//cout << "building:"<<++num << endl;
		toc();
		log(ps,viewPoints,points,bcs1,bcs2);
	}

	void log(const vector<Vector2d>& ps, const vector<Vector2d>& viewPoints, const vector<Vector2d>& points, const map<int, double>& bcs1, const map<int, double >& bcs2) const {
		const string filename = std::to_string(num);

		ofstream outfile00;
		outfile00.open("data/boundary_ori-" + filename + ".txt");
		outfile00.precision(20);
		for (auto t : ps) {
			outfile00 << t.x() << " " << t.y() << endl;
		}

		ofstream outfile0;
		outfile0.open("data/boundary-" + filename + ".txt");
		outfile0.precision(20);
		for (auto t : viewPoints) {
			outfile0 << t.x() << " " << t.y() << endl;
		}

		ofstream outfile1;
		outfile1.open("data/mesh-" + filename + ".txt");
		outfile1.precision(20);
		for (Vector2d point : points) {
			outfile1 << point.x() << " " << point.y() << endl;
		}
		outfile1.close();

		ofstream outfile2;
		outfile2.open("data/condition1-" + filename + ".txt");
		outfile2.precision(20);
		for (auto& bc : bcs1) {
			outfile2 << bc.first << " " << bc.second << endl;
		}
		outfile2.close();

		ofstream outfile3;
		outfile3.open("data/condition2-" + filename + ".txt");
		outfile3.precision(20);
		for (auto& bc : bcs2) {
			outfile3 << bc.first << " " << bc.second << endl;
		}
		outfile3.close();
	}

};

}} //namespace