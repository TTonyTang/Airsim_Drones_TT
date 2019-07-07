#pragma once

#include <Eigen/Eigen>
#include "ControlSystem.hpp"
#include <iostream>
using Eigen::MatrixXd;

namespace sxc {	namespace control_system {
	
class OptimalControlAnalytic{
public:
	static void lqg(const double q, const double r, const double qn, const double rn, const double qi, MatrixXd& A, MatrixXd& B, MatrixXd& C, MatrixXd& D) {
		const auto l = sqrt(qn / rn);
		const auto b = (q * q > 4 * r*qi ? 1 : -1)*sqrt(r*qi);
		const auto a = sqrt(r*(q - 2 * b));
		const auto k1 = a / r;
		const auto k2 = b / r;
		A.topLeftCorner(2, 2) = (-l - k1)*MatrixXd::Identity(2, 2);
		A.topRightCorner(2, 2) = (-k2)*MatrixXd::Identity(2, 2);
		A.bottomLeftCorner(2, 2) = MatrixXd::Zero(2, 2);
		A.bottomRightCorner(2, 2) = MatrixXd::Zero(2, 2);
		B.topLeftCorner(2, 2) = MatrixXd::Zero(2, 2);
		B.topRightCorner(2, 2) = l * MatrixXd::Identity(2, 2);
		B.bottomLeftCorner(2, 2) = MatrixXd::Identity(2, 2);
		B.bottomRightCorner(2, 2) = -MatrixXd::Identity(2, 2);
		C.topLeftCorner(2, 2) = -k1 * MatrixXd::Identity(2, 2);
		C.topRightCorner(2, 2) = -k2 * MatrixXd::Identity(2, 2);
		D = MatrixXd::Zero(2, 4);
	}

	static void lqg2(const double q, const double r, const double qn, const double rn, const double qi, MatrixXd& A, MatrixXd& B, MatrixXd& C, MatrixXd& D){
		const int nx = 2;
		const int nu = 2;
		const int ny = 2;
		const int nw = 2;
		const int nv = 2;
		const MatrixXd Q = q * MatrixXd::Identity(nx, nx);
		const MatrixXd R = r * MatrixXd::Identity(nu, nu);
		const MatrixXd Qn = qn * MatrixXd::Identity(nw, nw);
		const MatrixXd Rn = rn * MatrixXd::Identity(nv, nv);
		const MatrixXd Qi = qi * MatrixXd::Identity(ny, ny);
		MatrixXd AA = MatrixXd::Zero(2, 2);
		MatrixXd BB = MatrixXd::Identity(2, 2);
		MatrixXd CC = MatrixXd::Identity(2, 2);
		MatrixXd DD = MatrixXd::Zero(2, 2);
		const System sys(AA, BB, CC, DD);
		const System ret = ControlSystemUtil::lqg(sys,Q,R,Qn,Rn,Qi);
		A = ret.A;
		B = ret.B;
		C = ret.C;
		D = ret.D;
	}

	static void lqg3(const System& sys, const double q, const double r, const double qn, const double rn, const double qi, System& klqg) {
		const int nx = 2;
		const int nu = 2;
		const int ny = 2;
		const int nw = 2;
		const int nv = 2;
		const MatrixXd Q = q * MatrixXd::Identity(nx, nx);
		const MatrixXd R = r * MatrixXd::Identity(nu, nu);
		const MatrixXd Qn = qn * MatrixXd::Identity(nw, nw);
		const MatrixXd Rn = rn * MatrixXd::Identity(nv, nv);
		const MatrixXd Qi = qi * MatrixXd::Identity(ny, ny);
		//MatrixXd AA = MatrixXd::Zero(2, 2);
		//MatrixXd BB = MatrixXd::Identity(2, 2);
		//MatrixXd CC = MatrixXd::Identity(2, 2);
		//MatrixXd DD = MatrixXd::Zero(2, 2);
		//const System sys(AA, BB, CC, DD);
		const System ret = ControlSystemUtil::lqg(sys, Q, R, Qn, Rn, Qi);
		klqg.A = ret.A;
		klqg.B = ret.B;
		klqg.C = ret.C;
		klqg.D = ret.D;
	}
};

}}