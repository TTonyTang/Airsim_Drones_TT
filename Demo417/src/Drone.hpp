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

	
	Drone(){
		//client = new MultirotorRpcLibClient();
		//firmware = new Firmware(client);
	}
	

	

	//double ref_z = -12.0;

	
	void start(){


		firmware.connect();
		firmware.open();
	}

	
	void init(const double ref_z) {
		loadConfig();
		//cout << "init" << endl;
		firmware.moveToZ(ref_z, 5);


		//firmware.rotateToYaw(90);
		controller = new Controller(&firmware);



		cout << "ready" << endl;
	}

	
	void loadConfig(){
		std::ifstream i("mysettings.json");
		json j;
		i >> j;

		
		firmware.bcStrategy = j.at("bcStrategy").get<int>();
		firmware.meshStrategy = j.at("meshStrategy").get<int>();
		firmware.boundaryStrategy = j.at("boundaryStrategy").get<int>();
		firmware.maxDepth = j.at("maxDepth").get<double>();
		firmware.maxDist = j.at("maxDist").get<double>();
		firmware.minGap = j.at("minGap").get<double>();
	

		//cout << firmware.boundaryStrategy << ":" << firmware.meshStrategy << ":" << firmware.bcStrategy << ":" << firmware.maxDepth << ":" << firmware.maxDist << endl;
	}

	
	void task(const Vector2d& target, const double speed) const {
		cout << "before drone start 1" << endl;
		controller->start(target, speed);
		cout << "drone start ready 8" << endl;
		
		controller->waitToStop();
		
		
		if (controller != nullptr) {
			controller->stop();
		}
		
	}

	
	void stop(){
		
		firmware.close();
		firmware.disconnect();
		
	}

	

	//Firmware firmware;
	
	Firmware firmware;

	Controller* controller = nullptr;

private:

	MultirotorRpcLibClient * client = nullptr;

};