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

	Firmware(){}

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

	/*
	Firmware* moveByAngleZAsync(const float pitch, const float roll, const float yaw, const float ref_z){
		client->moveByAngleZAsync(pitch, roll, ref_z, yaw, max_duration);
		return this;
	}
	*/

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

	Firmware* open() {
		client->enableApiControl(true);
		client->armDisarm(true);
		client->takeoffAsync(2.0f)->waitOnLastTask();
		return this;
	}

	Firmware* close(){
		client->landAsync()->waitOnLastTask();
		client->armDisarm(false);
		client->enableApiControl(false);
		return this;
	}

	//Airsim bug
	Firmware* moveToZ(const double z, const double speed) {
		const auto timeout = static_cast<float>(fabs(z - getZ()) / speed + 2);
		client->moveToZAsync(static_cast<float>(z), static_cast<float>(speed), timeout)->waitOnLastTask();
		return this;
	}

	//Airsim bug
	Firmware* rotateToYaw(const double yaw) {
		const auto angle = static_cast<float>(yaw * M_PI / 180);
		client->moveByAngleZAsync(0, 0, static_cast<float>(getZ()), angle, 5)->waitOnLastTask();
		return this;
	}

	//Airsim bug
	Firmware* hoverAsync() {
		client->moveByVelocityZAsync(0,0, static_cast<float>(getZ()), 0.1f);
		return this;
	}

	double getZ() const {
		return client->getMultirotorState().getPosition().cast<double>().z();
	}

	void get2DState(Vector2d& pos, double& yaw, Vector2d& v) const {
		auto state = client->getMultirotorState();
		pos = state.getPosition().cast<double>().head<2>();
		v = state.kinematics_estimated.twist.linear.cast<double>().head<2>();
		yaw = static_cast<double>(VectorMath::getYaw(state.getOrientation()));
	}

	/*
	void setDisturbance(const double dist) const {
		Pose pose = client->simGetVehiclePose();
		const double yaw = VectorMath::getYaw(pose.orientation);
		pose.position.x() += float(dist * sin(yaw));
		pose.position.y() += float(-dist * cos(yaw));
		client->simSetVehiclePose(pose, true);
	}
	*/

	/*
	void getBoundary(Vector2d& pos, double& yaw, vector<Vector2d>& boundary) const{
		const auto ret = client->simGetBoundary();
		pos = ret.pos.cast<double>().head<2>();

		//cout <<pos<< endl;
		for(const auto& p: ret.boundary){
			const Vector2d b = p.cast<double>().head<2>();
			//cout << double((b - pos).norm()) << endl;
			boundary.emplace_back(b);
		}

		
		const auto N = int(boundary.size());
		yaw = static_cast<double>(VectorMath::getYaw(client->getMultirotorState().getOrientation()));
		const Vector2d direction(cos(yaw),sin(yaw));
		const auto targetTheta = atan2(direction.y(), direction.x());
		const double alpha = M_PI * 2 / 3;
		const int start = int(field_building::FieldMathUtil::regularAngle(targetTheta + alpha) / (2 * M_PI)*N);
		int end = int(field_building::FieldMathUtil::regularAngle(targetTheta - alpha) / (2 * M_PI)*N);
		end = start > end ? end + N : end;
		for (int i = start; i <= end; i++) {
			boundary[i%N] = pos + (boundary[i%N] - pos).normalized()*maxDepth;
		}
		
	}
	*/

	
	void getBoundary(Vector2d& pos, vector<Vector2d>& boundary) const {
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(0.0, sn*maxDepth);
		const auto ret = client->simGetBoundary();
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

		/*
		const auto N = int(boundary.size());
		auto direction = client->getMultirotorState().kinematics_estimated.twist.linear.cast<double>().head<2>();
		const auto targetTheta = atan2(direction.y(), direction.x());
		const double alpha = M_PI * 2 / 3;
		const int start = int(field_building::FieldMathUtil::regularAngle(targetTheta + alpha) / (2 * M_PI)*N);
		int end = int(field_building::FieldMathUtil::regularAngle(targetTheta - alpha) / (2 * M_PI)*N);
		end = start > end ? end + N : end;
		for (int i = start; i <= end; i++) {
			boundary[i%N] = pos + (boundary[i%N] - pos).normalized()*maxDepth;
		}
		*/
	}
	
	

	/*
	void get2DState2(Vector2d& pos, double& yaw, Vector2d& v, Vector2d& a) const {
		auto state = client->getMultirotorState();
		pos = state.getPosition().cast<double>().head<2>();
		v = state.kinematics_estimated.twist.linear.cast<double>().head<2>();
		yaw = static_cast<double>(VectorMath::getYaw(state.getOrientation()));
		a = state.kinematics_estimated.accelerations.linear.cast<double>().head<2>();
	}
	*/

	Vector3d getPosition() const{
		return client->getMultirotorState().getPosition().cast<double>();
	}

	/*
	double getYaw() const {
		return static_cast<double>(VectorMath::getYaw(client->getMultirotorState().getOrientation()));
	}
	*/
	/*
	double getYaw() const {
		return double(VectorMath::yawFromQuaternion(client->getMultirotorState().getOrientation()));
	}
	*/
	/*
	Quaterniond getOrientation() const {
		return client->getMultirotorState().kinematics_estimated.pose.orientation.cast<double>();
	}
	Vector3d getLinearVelocity() const {
		return client->getMultirotorState().kinematics_estimated.twist.linear.cast<double>();
	}
	Vector3d getAngularVelocity() const {
		return client->getMultirotorState().kinematics_estimated.twist.angular.cast<double>();
	}
	Vector3d getLinearAcceleration() const {
		return client->getMultirotorState().kinematics_estimated.accelerations.linear.cast<double>();
	}
	Vector3d getAngularAcceleration() const {
		return client->getMultirotorState().kinematics_estimated.accelerations.linear.cast<double>();
	}
	*/

	//double ref_speed = 2.0;
	
	double g = 9.8;
	double m = 1.0;//from Airsim
	double k = 1.3f / 4.0f;//from Airsim
	double maxDepth = 20.0;
	double sn = 0.05;
	//double ref_z = float(-12.0);

	double positionTolerance = 1.0;
	double velocityTolerance = 1.0;

	int bcStrategy = 1;
	int meshStrategy = 1;
	int boundaryStrategy = 1;
	double maxDist = 5 * maxDepth;
	double minGap = 2;
private:

	float max_duration = 0.1f;
	MultirotorRpcLibClient* client = nullptr;

};

}} //namespace