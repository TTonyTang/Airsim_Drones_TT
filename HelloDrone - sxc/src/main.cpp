// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//#include "common/common_utils/StrictMode.hpp"
#include <cstdlib>
#include "PathPlaningController/ControlSystem/OptimalControlAnalytic.hpp"
#include "PathPlaningController/BEM/BEM2DLinear.hpp"
#include "PathPlaningController/BEM/BEM2DConstant.hpp"

/*
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
STRICT_MODE_ON
*/

//#include "Drone.hpp"
/*
int main() 
{
	//auto qn = Controller::getQuaternion(Vector2d::Zero(),9.8);
	//cout << qn.x() << "," << qn.y() << "," << qn.z() << "," << qn.w() << endl;
	//float pitch, roll, yaw;
	//VectorMath::toEulerianAngle(qn.cast<float>(), pitch, roll, yaw);
	//cout << pitch << "," << roll << "," << yaw << endl;
	//testBEM3();
	
	
	//time_point time = clk::now();
	//sleep(1234);
	//time_point now = clk::now();

	//cout << "time:" << double(time.time_since_epoch().count()) << endl;
	//cout << "now:" << double(now.time_since_epoch().count()) << endl;
	//cout << "dt" << double(std::chrono::duration<double>(now - time).count())<< endl;

	
	const double q = 1;
	const double r = 0.1;
	const double qn = 0.01;
	const double rn = 0.1;
	const double qi = 100;
	MatrixXd A1(4, 4), B1(4, 4), C1(2, 4), D1(2, 4);
	MatrixXd A2(4, 4), B2(4, 4), C2(2, 4), D2(2, 4);
	sxc::control_system::OptimalControlAnalytic::lqg(q,r,qn,rn,qi,A1,B1,C1,D1);
	sxc::control_system::OptimalControlAnalytic::lqg2(q, r, qn, rn, qi, A2, B2, C2, D2);
	std::cout << A1 << std::endl << B1 << std::endl << C1 << std::endl << D1 << std::endl << std::endl;
	std::cout << A2 << std::endl << B2 << std::endl << C2 << std::endl << D2 << std::endl << std::endl;
	*/
	
	//testBEM3();
	/*
	cout << "God" << endl;
	MultirotorRpcLibClient* client = new MultirotorRpcLibClient();
	Drone drone;
	drone.start();
	drone.init(-12.0);
	drone.task(Vector2d(40, 60),2.0);
	
	
	system("pause");
    return 0;
}

*/


/*
void print(const std::tuple<double, Eigen::Vector2d>& tuple) {
	std::cout << std::get<0>(tuple) << "\t" << std::get<1>(tuple)(0) << "\t" << std::get<1>(tuple)(1) << std::endl;
}

template<class T>
void test(sxc::bem::BEM2D<T>& bemcase){
	const std::vector<Eigen::Vector2d> mesh{
		Eigen::Vector2d(0.0, 0.0),
		Eigen::Vector2d(1.0, 0.0),
		Eigen::Vector2d(1.0, 1.0),
		Eigen::Vector2d(0.0, 1.0) };
	bemcase.setMesh(mesh);
	const std::map<int, double> bc1{
		std::pair<int, double>(0,0),
		//std::pair<int, double>(1,50),
		std::pair<int, double>(2,100),
		//std::pair<int, double>(3,200)
		};
	const std::map<int, double> bc2;
	bemcase.setBoundaryCondition(bc1, bc2);
	bemcase.solve();
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.5, 0.5)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.0, 0.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.5, 0.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(1.0, 0.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(1.0, 0.5)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(1.0, 1.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.5, 1.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.0, 1.0)));
	print(bemcase.getPotentialAndFlux(Eigen::Vector2d(0.0, 0.5)));
}

int main()
{
	sxc::bem::BEM2DConstant bemcase1{};
	sxc::bem::BEM2DConstantAnalytical bemcase2{};
	sxc::bem::BEM2DLinear bemcase3{};
	test(bemcase1);
	std::cout << "--------------" << std::endl;
	test(bemcase2);
	std::cout << "--------------" << std::endl;
	test(bemcase3);
	system("pause");
}
*/