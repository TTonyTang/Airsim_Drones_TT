// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <utility>
#include "SimpleShell.hpp"
#include <common/Common.hpp>
#include "common/common_utils/Utils.hpp"
#include "common/common_utils/AsyncTasker.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "Drone.hpp"
#include "json.hpp"

//#include <rpc.h>

namespace msr {
	namespace airlib {

		using namespace std;
		using namespace common_utils;

		struct CommandContext {
		public:
			CommandContext(){}

			//创建无人机对象
			Drone drone;	//创建对象同时也开始控制drone
			AsyncTasker tasker;

	//		void sleep_for(TTimeDelta wall_clock_dt) const
	//		{
		//		clock_->sleep_for(wall_clock_dt);
	//		}

		private:
			ClockBase * clock_ = ClockFactory::get();
		};

		struct xyConfigCoordinate {
			double x;
			double y;
		};
		xyConfigCoordinate loadConfigCoordinate() {
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

		using DroneCommandParameters = SimpleShell<CommandContext>::ShellCommandParameters;
		using DroneCommandSwitch = SimpleShell<CommandContext>::ShellCommandSwitch;


		class DroneCommand : public SimpleShell<CommandContext>::ShellCommand {
		public:
			DroneCommand(std::string name, std::string help)
				: ShellCommand(name, help)
			{
			}

		};

		class InitCommand : public DroneCommand {
		public:
			InitCommand() : DroneCommand("Init", "Get things ready")
			{
				this->addSwitch({ "-z", "-7.0", "z position in meters (default -12.0)" });
				//theName	defaultValue	helpText
				//添加进指令表(unordered_map)
			}

			//执行
			bool execute(const DroneCommandParameters& params) override
			{
				const float z = getSwitch("-z").toFloat();	//通过z找到这条指令 

				CommandContext* context = params.context;
				context->drone.start();
				context->drone.init(static_cast<double>(z));
				return false;
			}
		};

		class MoveToTargetCommand : public DroneCommand {
		public:
			MoveToTargetCommand() : DroneCommand("MoveToTarget", "Move To Target")
			{
				this->addSwitch({ "-x", "0", "x position in meters (default 0)" });
				this->addSwitch({ "-y", "0", "y position in meters (default 0)" });
				this->addSwitch({ "-v", "0.5", "Velocity to move at (default 0.5 meters per second)" });
			}

			bool execute(const DroneCommandParameters& params) override
			{
				const float x = getSwitch("-x").toFloat();
				const float y = getSwitch("-y").toFloat();
				const float velocity = getSwitch("-v").toFloat();

				CommandContext* context = params.context;
				context->drone.task(Vector2d(x, y), velocity);
				return false;
			}
		};

		class MoveToCommand : public DroneCommand {
		public:
			MoveToCommand() : DroneCommand("MoveTo", "Move To")
			{
				this->addSwitch({ "-x", "0", "x position in meters (default 0)" });
				this->addSwitch({ "-y", "0", "y position in meters (default 0)" });
				this->addSwitch({ "-v", "0.5", "Velocity to move at (default 0.5 meters per second)" });
			}

			bool execute(const DroneCommandParameters& params) override
			{
				const float x = getSwitch("-x").toFloat();
				const float y = getSwitch("-y").toFloat();
				const float velocity = getSwitch("-v").toFloat();

				CommandContext* context = params.context;
				context->drone.task(Vector2d(x, y), velocity);
				return false;
			}
		};

		class GetPositionCommand : public DroneCommand {
		public:
			GetPositionCommand() : DroneCommand("Pos", "Get the current position")
			{
			}

			bool execute(const DroneCommandParameters& params) override
			{
				CommandContext* context = params.context;

				auto pos1 = context->drone.firmware.getPosition();
				//auto pos2 = context->drone.firmware2.getPosition();
				//auto biancha = loadConfigCoordinate();
				std::cout << "Drone1 Local position: x=" << pos1.x() << ", y=" << pos1.y() << ", z=" << pos1.z() << std::endl;
				//std::cout << "Drone2 Local position: x=" << pos2.x()+ biancha.x << ", y=" << pos2.y()+biancha.y << ", z=" << pos2.z() << std::endl;


				//auto pos = context->drone.firmware.getPosition();
				//std::cout << "Local position: x=" << pos.x() << ", y=" << pos.y() << ", z=" << pos.z() << std::endl;
				
				//auto pos1 = context->drone.firmware.getPosition();
				//auto pos2 = context->drone.firmware.getPosition2();
				//auto biancha = loadConfigCoordinate();
				//std::cout << "Drone1 Local position: x=" << pos1.x() << ", y=" << pos1.y() << ", z=" << pos1.z() << std::endl;
				//std::cout << "Drone2 Local position: x=" << pos2.x()+ biancha.x << ", y=" << pos2.y()+biancha.y << ", z=" << pos2.z() << std::endl;
				return false;
			}
		};


		

	}
} //namespace


/*
void printUsage() {
	std::cout << "Usage: DroneServer [-server 127.0.0.1]" << std::endl;
	std::cout << "The default server address is 127.0.0.1, but use the -server option to specify a different address for the server" << std::endl;
}
*/



int main(int argc, const char *argv[]) {

	using namespace msr::airlib;
	

	CommandContext command_context;

	SimpleShell<CommandContext> shell("==||=> ");

	shell.showMessage(R"(
        Welcome to DroneShell 1.0.
        This is a test for autonomous navigation base on Laplacian potential field.
        Type ? for help.
        Powered by Airsim, Microsoft Research (c) 2017.
    )");

	//command_context.client.confirmConnection();

	InitCommand init;
	MoveToTargetCommand moveToTarget;
	GetPositionCommand getPosition;

	MoveToCommand moveTo;
	shell.addCommand(moveTo);

	shell.addCommand(init);
	shell.addCommand(moveToTarget);
	shell.addCommand(getPosition);


	while (!shell.readLineAndExecute(&command_context)) {
	}

	return 0;
}
