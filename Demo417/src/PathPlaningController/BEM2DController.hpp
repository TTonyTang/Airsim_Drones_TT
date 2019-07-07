#pragma once

#include "Common.hpp"
#include "ControlSystem/ControlSystem.hpp"
#include "Firmware.hpp"
#include "FieldBuilding.hpp"
#include "ControlSystem/OptimalControlAnalytic.hpp"

namespace sxc { namespace path_planning_controller {

using control_system::System;
using control_system::ControlSystemUtil;

using std::thread;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using clk = std::chrono::high_resolution_clock;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Vector4d = Eigen::Matrix< double, 4, 1 >;
using time_point = std::chrono::time_point<clk>;
using duration = std::chrono::duration<double>;

class BEM2DController{

public:

	BEM2DController(Firmware* firmware) : firmware(firmware){
		const double q = 1;
		const double r = 1;
		const double qn = 0.01;
		const double rn = 0.1;
		const double qi = 100;
		//A = MatrixXd(4, 4);
		//B = MatrixXd(4, 4);
		//C = MatrixXd(2, 4);
		//D = MatrixXd(2, 4);
		MatrixXd AA = firmware->k/firmware->m*MatrixXd::Identity(2, 2);
		MatrixXd BB = MatrixXd::Identity(2, 2);
		MatrixXd CC = MatrixXd::Identity(2, 2);
		MatrixXd DD = MatrixXd::Zero(2, 2);
		const System sys(AA,BB,CC,DD);
		control_system::OptimalControlAnalytic::lqg3(sys, q, r, qn, rn, qi, klqg);
	};

	~BEM2DController() = default;

	void start(const Vector2d& target, const double speed) {
		cout << "before controller start 2" << endl;
		flag = true;
		target_ = target;
		speed_ = speed;
		//firmware->ref_speed = ref_speed;
		ref_z = firmware->getZ();
		field_building = new FieldBuilding(firmware);
		field_building->start(target_,speed_);
		//auto v = firmware->getLinearVelocity().head<2>();
		xe << 0, 0, 0, 0;
		thd = thread(&BEM2DController::run, this);
		cout << "controller start ready 7" << endl;
	}

	void stop() {
		flag = false;
		if (thd.joinable()) {
			thd.join();
		}
	}

	void run() {
		vector<Vector2d> ps;	//位置
		vector<Vector2d> vs;	//实时速度
		vector<Vector2d> rs;	//此时应该的速度

		cout << "controller runing 6" << endl;
		time = clk::now();
		double yaw_real = 0.0;
		Vector2d pos = Vector2d::Zero();
		Vector2d v = Vector2d::Zero();
		int num = 0;
		//std::default_random_engine generator;
		//std::normal_distribution<double> distribution(0.0, 3.0);
		//const int N = 1000;
		while (flag) {
			
			firmware->get2DState(pos, yaw_real, v);
			ps.emplace_back(pos);
			vs.emplace_back(v);
			if ((target_ - pos).norm()>firmware->positionTolerance || v.norm()>firmware->velocityTolerance) {
				Vector2d r = field_building->getFlux(pos);
				const auto dist = (target_ - pos).norm();	//dist距离
				if (dist < firmware->positionTolerance) {
					r = Vector2d::Zero();
				}
				double scale = speed_;
				/*if (dist<firmware->maxDepth) {
					scale = std::max(dist / firmware->maxDepth*speed_, firmware->velocityTolerance / 2);
				}*/
				r *= scale;
				rs.emplace_back(r);
				period(r, pos, yaw_real,v);
			}else {
				flag = false;
			}
			num++;
		}
		firmware->hoverAsync();
		if (field_building != nullptr) {
			field_building->stop();
			delete field_building;
			field_building = nullptr;
		}
		cout << "Controller iterations:" << num << endl;
		cv.notify_all();

		ofstream outfileps;
		outfileps.open("data/record data/ps.txt");
		outfileps.precision(20);
		for (auto& p : ps) {
			outfileps << p.x() << " " << p.y() << endl;
		}
		outfileps.close();

		ofstream outfilevs;
		outfilevs.open("data/record data/vs.txt");
		outfilevs.precision(20);
		for (auto& vel : vs) {
			outfilevs << vel.x() << " " << vel.y() << endl;
		}
		outfilevs.close();

		ofstream outfilers;
		outfilers.open("data/record data/rs.txt");
		outfilers.precision(20);
		for (auto& rv : rs) {
			outfilers << rv.x() << " " << rv.y() << endl;
		}
		outfilers.close();
	}

	void waitToStop(){
		std::unique_lock<std::mutex> lck{ mt };
		cv.wait(lck, [this]() {return !this->flag; });
		stop();
	}

private:

	Vector2d target_ = Vector2d::Zero();
	double speed_ = 1.0;
	double ref_z = -12.0;

	thread thd;
	bool flag = false;
	std::condition_variable cv;
	std::mutex mt;

	Firmware* firmware;
	FieldBuilding* field_building = nullptr;

	System klqg;
	//MatrixXd A;
	//MatrixXd B;
	//MatrixXd C;
	//MatrixXd D;

	Vector4d xe;
	time_point time; //initial when start

	Vector2d lqgController(const double dt, const Vector4d& ry) {
		const auto xedot = klqg.A*xe + klqg.B*ry;
		xe = xe + xedot * dt;
		return klqg.C*xe + klqg.D*ry;
	}

	void period(const Vector2d& r, const Vector2d& pos, const double yaw_real, const Vector2d& v){

		const time_point now = clk::now();
		const double dt = double(duration(now - time).count());
		time = now;

		Vector4d ry;
		ry << r, v;
		Vector2d u = lqgController(dt,ry);	//得出的加速度

		//加速度不要太大 30度
		if(u.norm()>firmware->g/sqrt(3)){
			u=u.normalized()*firmware->g / sqrt(3);
		}
		
		double desiredYaw;
		if (almost_zero(v.norm())) {
			desiredYaw = yaw_real;
		} else {
			desiredYaw = atan2(v.y(), v.x());
		}

		const auto q = getQuaternion(u, firmware->g, desiredYaw);

		//	控制无人机移动 姿态四元数+速度
		firmware->moveByQuaternionZAsync(q, ref_z);
	}

	//旋转矩阵转换成姿态四元数
	static Quaterniond getQuaternion(const Vector2d& a, const double g, const double desiredYaw) {
		Vector3d z(-a.x(), -a.y(), g);
		z.normalize();
		const auto vx = cos(desiredYaw);
		const auto vy = sin(desiredYaw);
		Vector3d x(vx, vy, (vx*a.x() + vy * a.y()) / g);
		x.normalize();
		const auto y = z.cross(x);
		Matrix3d rotation;
		rotation << x, y, z;
		return Quaterniond(rotation);
	}

};

}} //namespace