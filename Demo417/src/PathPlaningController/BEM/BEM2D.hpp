#pragma once

#include "BEMMathUtil.hpp"

//BEM±ß½çÔª·¨

namespace sxc { namespace bem {

struct Element {
	virtual ~Element() = default;
	static const int nodesNumPerEle;
	double length = 0.0;
	virtual double pointOnEle(const Vector2d& x) const = 0;
	//virtual double getT(const Vector2d& x) const = 0;
	virtual Vector2d getPoint(double t) const = 0;
	virtual	Vector2d getN(double t) const = 0;
	virtual double getU(double t) const = 0;
	virtual double getQ(double t) const = 0;
	//virtual double getFlux(double t) const = 0;
};

template<typename T>
class BEM2D{

public:

	virtual ~BEM2D() = default;

	void setMesh(const vector<Vector2d>& mesh) {
		this->elements.clear();
		this->nodesNum = int(mesh.size());
		for (auto i = 0; i < this->nodesNum; i += T::nodesNumPerEle) {
			this->elements.push_back(T(mesh, i));
		}
	}

	void setBoundaryCondition(const map<int, double>& bcs1, const map<int, double>& bcs2) {
		this->baseUs.setZero(this->nodesNum);
		this->baseQs.setZero(this->nodesNum);
		this->mask.setOnes(this->nodesNum);
		for (auto& bc : bcs1) {
			this->baseUs(bc.first) = bc.second;
			this->mask(bc.first) = false;
		}
		for (auto& bc : bcs2) {
			this->baseQs(bc.first) = bc.second;
			this->mask(bc.first) = true;
		}
	}

	virtual void solve() = 0;

	double getPotential(const Vector2d& x) const {
		/*
		for (const auto& ele : this->elements) {
			const double t = ele.pointOnEle(x);
			if (t >= -1 && t <= 1) {
				return ele.getU(t);
			}
		}
		*/
		return getInnerPotential(x);
	}

	Vector2d getFlux(const Vector2d& x) const {
		return getInnerFlux(x);
	}

	tuple<double, Vector2d> getPotentialAndFlux(const Vector2d& x) const {
		/*
		for (const auto& ele : this->elements) {
			const double t = ele.pointOnEle(x);
			if (t >= -1 && t <= 1) {
				//return ele.getFlux(t);
				if (sxc::bem::almost_zero(ele.getQ(t))) {
					return std::make_tuple(ele.getU(t), getInnerFlux(ele.getPoint(t)));
				}
				else {
					return std::make_tuple(ele.getU(t), -ele.getQ(t)*ele.getN(t));
				}
			}
		}
		*/
		return getInnerPotentialAndFlux(x);
	}

protected:

	vector<T> elements{};

	VectorXd baseUs{};

	VectorXd baseQs{};

	VectorXi mask{};

	int nodesNum = 0;

	static tuple<VectorXd, VectorXd> solveLinearAlgebra(const MatrixXd& H, const MatrixXd& G, const VectorXd& u, const VectorXd& q, const VectorXi& mask){
		MatrixXd tempH = H;
		MatrixXd tempG = G;
		VectorXd tempu = u;
		VectorXd tempq = q;
		vector<int> indices;

		for(int i=0;i<mask.size();i++){
			if(mask(i)){
				indices.push_back(i);
			}
		}

		for(int i : indices){
			swapColumns(tempH, tempG, i);
			swapConditions(tempu, tempq, i);
		}

		//cout << tempG.row(0) << endl;
		tempq = tempG.colPivHouseholderQr().solve(tempH * tempu);
		//cout << tempq << endl;
		for (int i : indices) {
			swapConditions(tempu, tempq, i);
		}
		return std::make_tuple(tempu, tempq);
	}

	virtual double getInnerPotential(const Vector2d & x) const {
		double u = 0;
		for (const auto& elej : this->elements) {
			u += intUQ(x, elej) - intQU(x, elej);
		}
		return u / 2 / M_PI;
	}

	virtual Vector2d getInnerFlux(const Vector2d & x) const {
		Vector2d flux = Vector2d::Zero();
		for (const auto& elej : this->elements) {
			flux(0) += intDUQ(1, x, elej) - intDQU(1, x, elej);
			flux(1) += intDUQ(2, x, elej) - intDQU(2, x, elej);
		}
		return -flux / 2 / M_PI;
	}

	virtual tuple<double, Vector2d> getInnerPotentialAndFlux(const Vector2d & x) const {
		return std::make_tuple(getInnerPotential(x), getInnerFlux(x));
	}

private:

	static double intUQ(const Vector2d& x, const T& elej) {
		const auto uq = [x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto ustar = -log(r);
			const auto q = elej.getQ(t);
			return ustar * q * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(uq);
	}

	static double intQU(const Vector2d& x, const T& elej) {
		const auto qu = [x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto D2 = elej.getN(t).dot(y - x);
			const auto qstar = -D2 / (r * r);
			const auto u = elej.getU(t);
			return qstar * u * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(qu);
	}

	static double intDUQ(const int k, const Vector2d& x, const T& elej) {
		const auto duq = [k, x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto drk = -(x(k - 1) - y(k - 1)) / r;
			const auto dukstar = drk / r;
			const auto q = elej.getQ(t);
			return dukstar * q * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(duq);
	}

	static double intDQU(const int k, const Vector2d& x, const T& elej) {
		const auto dqu = [k, x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto drk = (x(k - 1) - y(k - 1)) / r;
			const auto drl = (x(2 - k) - y(2 - k)) / r;
			const auto normal = elej.getN(t);
			const auto nk = normal(k - 1);
			const auto nl = normal(2 - k);
			const auto dqkstar = -((2 * drk * drk - 1) * nk + 2 * drk * drl * nl) / (r * r);
			const auto u = elej.getU(t);
			return dqkstar * u * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(dqu);
	}

	static void swapColumns(MatrixXd& H, MatrixXd& G, const int i) {
		const MatrixXd temp = H.col(i);
		H.col(i) = -G.col(i);
		G.col(i) = -temp;
	}

	static void swapConditions(VectorXd& u, VectorXd& q, const int i) {
		const double temp = u[i];
		u[i] = q[i];
		q[i] = temp;
	}

};

}} //namespace