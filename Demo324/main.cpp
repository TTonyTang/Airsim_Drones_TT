// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>



int main() 
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    
    try {
		//连接
        client.confirmConnection();

		//图像
        std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
        vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthPlanner, true) };
        const vector<ImageResponse>& response = client.simGetImages(request);
        std::cout << "# of images received: " << response.size() << std::endl;
		//存几个图像

        if (response.size() > 0) {
            //存图路径
			std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl; 
            std::string path;
            std::getline(std::cin, path);

            for (const ImageResponse& image_info : response) {
                std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
                std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

                if (path != "") {
                    std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
                    if (image_info.pixels_as_float) {//是否是浮点储存
                        Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
                            file_path + ".pfm");
                    }
                    else {
                        std::ofstream file(file_path + ".png", std::ios::binary);
                        file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                        file.close();
                    }
                }
            }
        }

        std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
        client.enableApiControl(true);
        client.armDisarm(true);

		//如果脚本睡眠时间超过100毫秒，PX4也会使无人机脱离API控制（也称为外接控制），
		//这也是一个安全功能，所以如果你在调试器中，
		//例如你停止在其中一条线上，它会阻止AirSim发送offboard命令

        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
		//等待此任务完成
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5)); //等待执行指令5秒
		client.hoverAsync()->waitOnLastTask();	//	hover模式？

        std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl; std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.enableApiControl(true); 
        auto position = client.getMultirotorState().getPosition();
        float z = position.z(); // current position (NED coordinate system).  
        const float speed = 3.0f;
        const float size = 10.0f; 
        const float duration = size / speed;
		//ForwardOnly前部指向行驶方向  MaxDegreeOfFreedom向任何方向前进
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        //偏航 true 度/秒角速度； false 度角速度
		YawMode yaw_mode(true, 0);

        std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, 2, z, 1, driveTrain, yaw_mode)->waitOnLastTask();
		//std::this_thread::sleep_for(std::chrono::duration<double>(float(1)));


        //std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        //client.moveByVelocityZAsync(0, 5, z, 2, driveTrain, yaw_mode);
        //std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        //std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        //client.moveByVelocityZAsync(-speed, 0, z, duration, driveTrain, yaw_mode);
        //std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        //std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        //client.moveByVelocityZAsync(0, -speed, z, duration, driveTrain, yaw_mode);
        //std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hoverAsync()->waitOnLastTask();
		client.enableApiControl(true);	//!!!非常重要 call to after the offboard operation succeeded

        std::cout << "Press Enter to land" << std::endl; std::cin.get();
        client.landAsync()->waitOnLastTask();

        std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
        client.armDisarm(false);

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}
