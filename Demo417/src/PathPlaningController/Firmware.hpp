#pragma once

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include "FieldBuilding/FieldMathUtil.hpp"

namespace sxc { namespace path_planning_controller {

using msr::airlib::MultirotorRpcLibClient;
using msr::airlib::VectorMath;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Quaterniond;
using std::vector;
using std::tuple;
using std::make_tuple;

class Firmware{

public:
	//std::string dronename="Drone2";

	Firmware(){}

	

	//是用这个来移动无人机
	//done
	Firmware* moveByQuaternionZAsync(const Quaterniond& q, const double ref_z) {
		float pitch, roll, yaw;
		VectorMath::toEulerianAngle(q.cast<float>(), pitch, roll, yaw);

		const float limit = M_PIf * 6;
		const float anglescaler = std::max(std::max(abs(pitch) / limit, abs(roll) / limit), 1.0f);
		pitch /= anglescaler;
		roll /= anglescaler;

		client->moveByAngleZAsync(pitch, roll, float(ref_z), yaw, max_duration);
		return this;
	}


	Firmware* connect() {
		client = new MultirotorRpcLibClient();
		client->confirmConnection();
		return this;
	}

	Firmware* disconnect(){
		delete client;
		client = nullptr;
		return this;
	}

	//done
	Firmware* open() {
		//client->enableApiControl(true,"Drone1");
		//client->enableApiControl(true, "Drone2");
		//client->armDisarm(true, "Drone1");
		//client->armDisarm(true, "Drone2");
		//client->takeoffAsync(2.0f, "Drone1")->waitOnLastTask();
		//client->takeoffAsync(2.0f, "Drone2")->waitOnLastTask();
		client->enableApiControl(true);
		client->armDisarm(true);
		client->takeoffAsync(2.0f)->waitOnLastTask();
		return this;
	}

	//done
	Firmware* close(){
		client->landAsync(60)->waitOnLastTask();
		client->armDisarm(false);
		client->enableApiControl(false);
		return this;
	}

	//done
	//Airsim bug
	Firmware* moveToZ(const double z, const double speed) {
		const auto timeout = static_cast<float>(fabs(z - getZ()) / speed + 2);
		
		client->moveToZAsync(static_cast<float>(z), static_cast<float>(speed), timeout, YawMode(), 1, 1)->waitOnLastTask();

		//client->moveToZAsync(static_cast<float>(z), static_cast<float>(speed), timeout)->waitOnLastTask();
		
		//client->moveToZAsync(static_cast<float>(z), static_cast<float>(speed), timeout, YawMode(),1,1,"Drone1")->waitOnLastTask();
		//client->moveToZAsync(static_cast<float>(z), static_cast<float>(speed), timeout, YawMode(),1,1,"Drone2")->waitOnLastTask();
		return this;
	}

	//done
	//Airsim bug
	Firmware* rotateToYaw(const double yaw) {
		const auto angle = static_cast<float>(yaw * M_PI / 180);
		client->moveByAngleZAsync(0, 0, static_cast<float>(getZ()), angle, 5)->waitOnLastTask();
		//client->moveByAngleZAsync(0, 0, static_cast<float>(getZ()), angle, 5)->waitOnLastTask();
		return this;
	}

	//done
	//Airsim bug
	Firmware* hoverAsync() {
		client->moveByVelocityZAsync(0,0, static_cast<float>(getZ()), 0.1f, DrivetrainType::MaxDegreeOfFreedom ,YawMode());
		//client->moveByVelocityZAsync(0, 0, static_cast<float>(getZ()), 0.1f);
		return this;
	}

	//done
	double getZ() const {
		return client->getMultirotorState().getPosition().cast<double>().z();
		//return client->getMultirotorState().getPosition().cast<double>().z();
	}

	//done
	void get2DState(Vector2d& pos, double& yaw, Vector2d& v) const {
		auto state = client->getMultirotorState();
		//auto state = client->getMultirotorState();
		pos = state.getPosition().cast<double>().head<2>();
		v = state.kinematics_estimated.twist.linear.cast<double>().head<2>();
		yaw = static_cast<double>(VectorMath::getYaw(state.getOrientation()));
	}

	//done
	void getBoundary(Vector2d& pos, vector<Vector2d>& boundary) const {
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(0.0, sn*maxDepth);
		const auto ret = client->simGetBoundary();
		//const auto ret = client->simGetBoundary();
		pos = ret.pos.cast<double>().head<2>();
		//cout <<pos<< endl;
		for (const auto& p : ret.boundary) {
			double noise = distribution(generator);
			noise = std::max(noise, -maxDepth/2);
			//const double noise = 0;
			Vector2d b = p.cast<double>().head<2>();
			auto dist = (b - pos).norm();
			dist += noise;
			b = pos + dist * ((b - pos).normalized());
			//cout << double((b - pos).norm()) << endl;
			boundary.emplace_back(b);
		}

		
	}
	
	
	/*
	Vector3d getPosition() const{
		return client->getMultirotorState("Drone1").getPosition().cast<double>();
	}
	Vector3d getPosition2() const {
		return client->getMultirotorState("Drone2").getPosition().cast<double>();
	}
	*/
	
	
	//done
	Vector3d  getPosition() const {
		return client->getMultirotorState().getPosition().cast<double>();
	}
	
	

	

	//double ref_speed = 2.0;
	
	double g = 9.8;
	double m = 1.0;//from Airsim
	double k = 1.3f / 4.0f;//from Airsim 空气阻力系数
	double maxDepth = 20.0;
	double sn = 0.05;	//噪声占的比例
	//double ref_z = float(-12.0);

	double positionTolerance = 1.0;
	double velocityTolerance = 1.0;

	int bcStrategy = 1;
	int meshStrategy = 1;
	int boundaryStrategy = 1;
	double maxDist = 2.5 * maxDepth;
	double minGap = 2;
private:

	float max_duration = 0.1f;
	MultirotorRpcLibClient* client = nullptr;

};

}} //namespace