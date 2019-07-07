#pragma once

#include <Eigen/Eigen>

//¿ØÖÆÏµÍ³

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::EigenSolver;
using Eigen::MatrixXcd;

namespace sxc { namespace control_system {

struct System {
	MatrixXd A, B, C, D;
	System(){}
	System(MatrixXd& A, MatrixXd& B, MatrixXd& C, MatrixXd& D) :A(A), B(B), C(C), D(D) {};
};

class ControlSystemUtil {

public:

	//Github:TakaHoribe/Riccati_Solver
	static MatrixXd solveCARE(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R) {
		int j = 0;
		const auto dim_x = A.rows();

		// set Hamilton matrix
		MatrixXd Ham = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
		Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

		// calc eigenvalues and eigenvectors
		EigenSolver<MatrixXd> Eigs(Ham);

		// extract stable eigenvectors into 'eigvec'
		MatrixXcd eigvec = MatrixXcd::Zero(2 * dim_x, dim_x);
		for (int i = 0; i < 2 * dim_x; ++i) {
			if (Eigs.eigenvalues()[i].real() < 0.) {
				eigvec.col(j++) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
			}
		}
		const MatrixXcd Vs_1 = eigvec.topLeftCorner(dim_x, dim_x);
		const MatrixXcd Vs_2 = eigvec.bottomLeftCorner(dim_x, dim_x);
		return (Vs_2 * Vs_1.inverse()).real();
	}

	static MatrixXd lqr(const System& sys, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& N) {
		const auto P = solveCARE(sys.A, sys.B, Q, R);
		return R.inverse()*(sys.B.transpose()*P + N.transpose());
	}

	static MatrixXd lqi(const System& sys, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& N) {
		const auto nx = sys.A.rows();
		const auto nu = sys.B.cols();
		const auto ny = sys.C.rows();
		MatrixXd A(nx + ny, nx + ny), B(nx + ny, nu), C(ny, nx + ny), D(ny, nu);
		A << sys.A, MatrixXd::Zero(nx, ny),
			-sys.C, MatrixXd::Zero(ny, ny);
		B << sys.B,
			-sys.D;
		C << sys.C, MatrixXd::Zero(ny, ny);
		D << sys.D;
		const System sys_aug(A, B, C, D);
		return lqr(sys_aug, Q, R, N);
	}

	static System kalman(const System& sys, const MatrixXd& Qn, const MatrixXd& Rn, const MatrixXd& Nn) {
		const auto nw = Qn.rows();
		const auto nx = sys.A.rows();
		const auto ny = sys.C.rows();
		const auto nuw = sys.B.cols();
		const auto nu = nuw - nw;
		const auto A = sys.A;
		const auto B = sys.B.leftCols(nu);
		const auto C = sys.C;
		const auto D = sys.D.leftCols(nu);
		const auto G = sys.B.rightCols(nw);
		const auto H = sys.D.rightCols(nw);

		const auto R_bar = Rn + H * Nn + Nn.transpose()*H.transpose() + H * Qn*H.transpose();
		const auto N_bar = G * (Qn*H.transpose() + Nn);
		const auto P = solveCARE(A.transpose(), C.transpose(), Qn, Rn);
		const auto L = (P*C.transpose() + N_bar)*R_bar.inverse();

		MatrixXd A_new(nx, nx), B_new(nx, nu + ny), C_new(nx + ny, nx), D_new(nx + ny, nu + ny);
		A_new << A - L * C;
		B_new << B - L * D, L;
		C_new << C,
			MatrixXd::Identity(nx, nx);
		D_new << D, MatrixXd::Zero(ny, ny),
			MatrixXd::Zero(nx, nu), MatrixXd::Zero(nx, ny);
		return System(A_new, B_new, C_new, D_new);
	}

	static System lqgtrack(const System& kest, const MatrixXd& K) {
		const auto nu = K.rows();
		const auto nx = kest.A.rows();
		const auto nxy = kest.C.rows();
		const auto ny = nxy - nx;
		const auto Kx = K.leftCols(nx);
		const auto Ki = K.rightCols(ny);
		const auto A_tilda = kest.A;
		const auto B_tilda = kest.B.leftCols(nu);
		const auto L = kest.B.rightCols(ny);
		MatrixXd A(nx + ny, nx + ny), B(nx + ny, ny + ny), C(nu, nx + ny), D(nu, ny + ny);
		A << A_tilda - B_tilda * Kx, -B_tilda * Ki,
			MatrixXd::Zero(ny, nx), MatrixXd::Zero(ny, ny);
		B << MatrixXd::Zero(nx, ny), L,
			MatrixXd::Identity(ny, ny), -MatrixXd::Identity(ny, ny);
		C << -K;
		D << MatrixXd::Zero(nu, ny + ny);
		return System(A, B, C, D);
	}

	static System lqg(const System& sys, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& N, const MatrixXd& Qn, const MatrixXd& Rn, const MatrixXd& Nn, const MatrixXd& Qi) {
		const auto nx = sys.A.rows();
		const auto ny = sys.C.rows();
		const auto nu = sys.B.cols();
		const auto nw = nx;
		const auto G = MatrixXd::Identity(nx, nw);
		const auto H = MatrixXd::Zero(ny, nw);

		MatrixXd QA = MatrixXd::Zero(nx + ny, nx + ny);
		QA.topLeftCorner(nx, nx) = Q;
		QA.bottomRightCorner(ny, ny) = Qi;

		MatrixXd NA = MatrixXd::Zero(nx + ny, nu);
		NA.topLeftCorner(nx, nu) = N;

		MatrixXd BG = MatrixXd::Zero(nx, nx + nu);
		BG.topLeftCorner(nx, nu) = sys.B;
		BG.topRightCorner(nx, nw) = G;

		MatrixXd DH = MatrixXd::Zero(ny, nx + nu);
		DH.topLeftCorner(ny, nu) = sys.D;
		DH.topRightCorner(ny, nw) = H;

		MatrixXd A = sys.A;
		MatrixXd C = sys.C;

		const System sys_aug(A, BG, C, DH);
		const System kest = kalman(sys_aug, Qn, Rn, Nn);
		const MatrixXd K = lqi(sys, QA, R, NA);
		return lqgtrack(kest, K);
	}

	static System lqg(const System& sys, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& Qn, const MatrixXd& Rn, const MatrixXd& Qi) {
		const auto N = MatrixXd::Zero(Q.rows(), R.cols());
		const auto Nn = MatrixXd::Zero(Qn.rows(), Rn.cols());
		return lqg(sys, Q, R, N, Qn, Rn, Nn, Qi);
	}

	static System lqg(const System& sys, const MatrixXd& QXU, const MatrixXd& QWV, const MatrixXd& QI) {
		const auto nx = sys.A.rows();
		const auto ny = sys.C.rows();
		const auto nu = sys.B.cols();
		const auto nw = nx;
		const auto nv = ny;
		const auto Q = QXU.topLeftCorner(nx, nx);
		const auto R = QXU.bottomRightCorner(nu, nu);
		const auto N = QXU.topRightCorner(nx, nu);

		const auto Qn = QWV.topLeftCorner(nw, nw);
		const auto Rn = QWV.bottomRightCorner(nv, nv);
		const auto Nn = QWV.topRightCorner(nw, nv);

		return lqg(sys, Q, R, N, Qn, Rn, Nn, QI);
	}

};

}} //namespace
