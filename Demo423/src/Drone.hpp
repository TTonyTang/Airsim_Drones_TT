#pragma once

#define _MATH_DEFINES_DEFINED

#include "PathPlaningController/BEM2DController.hpp"
#include "PathPlaningController/Firmware.hpp"
#include "json.hpp"
//#include <nlohmann/json.hpp>

using std::cout;
using std::endl;
using msr::airlib::MultirotorRpcLibClient;
using Controller = sxc::path_planning_controller::BEM2DController;
using sxc::path_planning_controller::Firmware;
using Eigen::Vector2d;
using json= nlohmann::json;

class Drone {

public:

	Drone() :firmware1("Drone1"), firmware2("Drone2") {
		//client = new MultirotorRpcLibClient();
		//firmware = new Firmware(client);
	}

	//double ref_z = -12.0;

	struct xyConfigCoordinate {
		double x;
		double y;
	};
	xyConfigCoordinate loadConfigCoordinate() const {
		std::ifstream i("C:/Users/zjdell/Documents/AirSim/settings.json");
		json j;
		i >> j;
		xyConfigCoordinate xy;
		xy.x = j.at("Vehicles").at("Drone2").at("X").get<double>();
		xy.y = j.at("Vehicles").at("Drone2").at("Y").get<double>();
		//cout << xy.x << endl;
		//cout << xy.y << endl;
		return xy;
	}


	//done
	void start(){
		firmware1.connect();
		firmware2.connect();
		firmware1.open();
		firmware2.open();

		//firmware.connect();
		//firmware.open();
	}

	//done
	void init(const double ref_z) {
		loadConfig();
		//cout << "init" << endl;
		//firmware.moveToZ(ref_z, 5);

		firmware1.moveToZ(ref_z, 5);
		firmware2.moveToZ(ref_z, 5);

		//firmware.rotateToYaw(90);
		//controller = new Controller(&firmware);

		controller1 = new Controller(&firmware1);
		controller2 = new Controller(&firmware2);

		cout << "ready" << endl;
	}

	//done
	void loadConfig(){
		std::ifstream i("mysettings.json");
		json j;
		i >> j;

		/*
		firmware.bcStrategy = j.at("bcStrategy").get<int>();
		firmware.meshStrategy = j.at("meshStrategy").get<int>();
		firmware.boundaryStrategy = j.at("boundaryStrategy").get<int>();
		firmware.maxDepth = j.at("maxDepth").get<double>();
		firmware.maxDist = j.at("maxDist").get<double>();
		firmware.minGap = j.at("minGap").get<double>();
		*/

		firmware1.bcStrategy = j.at("bcStrategy").get<int>();
		firmware1.meshStrategy = j.at("meshStrategy").get<int>();
		firmware1.boundaryStrategy = j.at("boundaryStrategy").get<int>();
		firmware1.maxDepth = j.at("maxDepth").get<double>();
		firmware1.maxDist = j.at("maxDist").get<double>();
		firmware1.minGap = j.at("minGap").get<double>();
		firmware2.bcStrategy = j.at("bcStrategy").get<int>();
		firmware2.meshStrategy = j.at("meshStrategy").get<int>();
		firmware2.boundaryStrategy = j.at("boundaryStrategy").get<int>();
		firmware2.maxDepth = j.at("maxDepth").get<double>();
		firmware2.maxDist = j.at("maxDist").get<double>();
		firmware2.minGap = j.at("minGap").get<double>();

		//cout << firmware.boundaryStrategy << ":" << firmware.meshStrategy << ":" << firmware.bcStrategy << ":" << firmware.maxDepth << ":" << firmware.maxDist << endl;
	}

	//done
	void task(const Vector2d& target, const double speed) const {
		cout << "before drone start 1" << endl;
		//controller->start(target, speed);

		auto biancha = loadConfigCoordinate();
		Vector2d target2 = Vector2d::Zero();;
		target2 << target.x()- biancha.x, target.y()- biancha.y;

		controller1->start(target, speed);
		controller2->start(target2, speed);
		cout << "drone start ready 8" << endl;
		
		//controller->waitToStop();
		controller1->waitToStop();
		controller2->waitToStop();
		
		/*
		if (controller != nullptr) {
			controller->stop();
		}
		*/
		
		if (controller1 != nullptr) {
			controller1->stop();
		}
		if (controller2 != nullptr) {
			controller2->stop();
		}
	}

	//done
	void stop(){
		/*
		firmware.close();
		firmware.disconnect();
		*/

		firmware1.close();
		firmware2.close();
		firmware1.disconnect();
		firmware2.disconnect();
	}

	
	

	//Firmware firmware;
	
	Firmware firmware1;
	Firmware firmware2;

	Controller* controller1 = nullptr;
	Controller* controller2 = nullptr;

private:

	MultirotorRpcLibClient * client = nullptr;

};