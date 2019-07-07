#pragma once

#include "BEMCommon.hpp"
#include "BEM2D.hpp"
#include "BEMMathUtil.hpp"

namespace sxc { namespace bem {

struct ConstantElement : Element{
	static const int nodesNumPerEle = 1;
	Vector2d p1;
	Vector2d p2;
	Vector2d node;
	Vector2d normal;
	Vector2d unit;
	double u = 0.0;
	double q = 0.0;
	//Vector2d flux;
	double length = 0.0;
	ConstantElement(const vector<Vector2d>& points, int i) {
		p1 = points[i];
		p2 = points[(i + 1) % points.size()];
		node = (p1 + p2) / 2;
		const Vector2d delta = p2 - p1;
		length = delta.norm();
		unit = delta / this->length;
		const Vector2d perp(delta.y(), -delta.x());
		normal = perp / this->length;
	}
	double pointOnEle(const Vector2d& x) const override {
		if(BEMMathUtil::isPointOnLine(p1, p2, x)){
			return 0;
		} else {
			return -2;
		}
	}
	/*
	double getT(const Vector2d& x) const override{
		return 0;
	}
	*/
	Vector2d getPoint(const double t) const override {
		return node + t * (p2 - node);
	}
	Vector2d getN(const double t) const override {
		return normal;
	}
	double getU(const double t) const override {
		return u;
	}
	double getQ(const double t) const override {
		return q;
	}
	/*
	Vector2d getFlux(const double t) const override {
		return flux;
	}
	*/
};

class BEM2DConstant : public BEM2D<ConstantElement> {

public:

	void solve() override {
		MatrixXd G(this->nodesNum, this->nodesNum);
		for (auto i = 0; i < this->nodesNum; i++) {
			for (auto j = 0; j < this->nodesNum; j++) {
				if (i == j) {
					const auto& elej = this->elements[j];
					G(i, j) = elej.length *(1 - log(elej.length / 2));
				}
				else {
					G(i, j) = Gxj(this->elements[i].node, this->elements[j]);
				}
			}
		}
		MatrixXd H(this->nodesNum, this->nodesNum);
		for (auto i = 0; i < this->nodesNum; i++) {
			for (auto j = 0; j < this->nodesNum; j++) {
				if (i == j) {
					H(i, j) = M_PI;
				}
				else {
					H(i, j) = Hxj(this->elements[i].node, this->elements[j]);
				}
			}
		}
		std::tie(this->baseUs, this->baseQs) = solveLinearAlgebra(H, G, this->baseUs, this->baseQs, this->mask);
		for (auto i = 0; i < this->nodesNum; i++) {
			this->elements[i].u = this->baseUs[i];
			this->elements[i].q = this->baseQs[i];
		}
		/*
		for (auto& ele : this->elements) {
			if (sxc::bem::almost_zero(ele.q)) {
				ele.flux = getInnerFlux(ele.node);
			}
			else {
				ele.flux = ele.q*ele.normal;
			}
		}
		*/
	}

protected:

	static double Gxj(const Vector2d& x, const ConstantElement& elej) {
		const auto g = [x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto u = -log(r);
			return u* elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(g);
	}

	static double Hxj(const Vector2d& x, const ConstantElement& elej) {
		const double D = elej.normal.dot(elej.p1 - x);
		const auto h = [x, elej, D](const double& t) {
			const auto y = elej.getPoint(t);
			const double r = (x - y).norm();
			return -D / (r * r) * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(h);
	}

};

class BEM2DConstantAnalytical : public BEM2D<ConstantElement> {

public:

	double getInnerPotential(const Vector2d & x) const override{
		double u = 0.0;
		double g, h;
		for (const auto& elej : elements) {
			GHxj(x, elej, g, h);
			u += g * elej.q - h * elej.u;
		}
		return u / 2 / M_PI;
	}

	Vector2d getInnerFlux(const Vector2d & x) const override {
		Vector2d flux = Vector2d::Zero();
		Vector2d du, dq;
		for (const auto& elej : elements) {
			DUDQxj(x, elej, du, dq);
			flux += du * elej.q - dq * elej.u;
		}
		return -flux / 2 / M_PI;
	}

	tuple<double, Vector2d> getInnerPotentialAndFlux(const Vector2d & x) const override {
		double u = 0.0;
		Vector2d flux = Vector2d::Zero();

		double g, h;
		Vector2d du, dq;

		for (const auto& elej : elements) {
			GHDUDQxj(x, elej, g,h, du, dq);
			u += g * elej.q - h * elej.u;
			flux += du * elej.q - dq * elej.u;
		}
		return std::make_tuple(u / 2 / M_PI, -flux / 2 / M_PI);
	}

	void solve() override {
		MatrixXd G(this->nodesNum, this->nodesNum);
		MatrixXd H(this->nodesNum, this->nodesNum);

		for (auto i = 0; i < this->nodesNum; i++) {
			const auto x = this->elements[i].node;
			for (auto j = 0; j < this->nodesNum; j++) {
				const auto& elej = this->elements[j];
				if (i == j) {
					G(i, j) = elej.length *(1 - log(elej.length / 2));
					H(i, j) = M_PI;
				} else {
					GHxj(x, elej, G(i, j), H(i, j));
				}
			}
		}
		std::tie(this->baseUs, this->baseQs) = solveLinearAlgebra(H, G, this->baseUs, this->baseQs, this->mask);
		for (auto i = 0; i < this->nodesNum; i++) {
			this->elements[i].u = this->baseUs[i];
			this->elements[i].q = this->baseQs[i];
		}
		/*
		for (auto& ele : this->elements) {
			if (sxc::bem::almost_zero(ele.q)) {
				ele.flux = getInnerFlux(ele.node);
			} else {
				ele.flux = ele.q*ele.normal;
			}
		}
		*/
	}

	static void GHxj(const Vector2d& x, const ConstantElement& elej, double& g, double& h) {
		const auto r1vector = elej.p1 - x;
		const auto r2vector = elej.p2 - x;
		const auto r1 = r1vector.norm();
		const auto r2 = r2vector.norm();
		const auto T1 = r1vector.dot(elej.unit);
		const auto T2 = r2vector.dot(elej.unit);
		const auto d = elej.normal.dot(elej.p1 - x);
		const auto dtheta = atan(T2 / d) - atan(T1 / d);
		g = (-dtheta *d + elej.length - T2 * log(r2) + T1 * log(r1));
		h = -dtheta;
	}

	static void DUDQxj(const Vector2d& x, const ConstantElement& elej, Vector2d& du, Vector2d& dq) {
		const auto r1vector = elej.p1 - x;
		const auto r2vector = elej.p2 - x;
		const auto r1 = r1vector.norm();
		const auto r2 = r2vector.norm();
		const auto T1 = r1vector.dot(elej.unit);
		const auto T2 = r2vector.dot(elej.unit);
		const auto d = elej.normal.dot(elej.p1 - x);
		const auto ynormal = elej.normal.array();
		const auto tnormal = elej.unit.array();
		const auto r1s = r1 * r1;
		const auto r2s = r2 * r2;
		const auto dtheta = atan(T2 / d) - atan(T1 / d);
		du = (dtheta*ynormal + log(r2 / r1)*tnormal).matrix();
		dq = (-(T2 / r2s - T1 / r1s)*ynormal + d * (1 / r2s - 1 / r1s)*tnormal).matrix();
	}

	static void GHDUDQxj(const Vector2d& x, const ConstantElement& elej, double& g, double& h, Vector2d& du, Vector2d& dq) {
		const auto r1vector = elej.p1 - x;
		const auto r2vector = elej.p2 - x;
		const auto r1 = r1vector.norm();
		const auto r2 = r2vector.norm();
		const auto T1 = r1vector.dot(elej.unit);
		const auto T2 = r2vector.dot(elej.unit);
		const auto d = elej.normal.dot(elej.p1 - x);
		const auto ynormal = elej.normal.array();
		const auto tnormal = elej.unit.array();
		const auto r1s = r1 * r1;
		const auto r2s = r2 * r2;
		const auto dtheta = atan(T2 / d) - atan(T1 / d);
		g = (-dtheta * d + elej.length - T2 * log(r2) + T1 * log(r1));
		h = -dtheta;
		du = (dtheta*ynormal + log(r2 / r1)*tnormal).matrix();
		dq = (-(T2 / r2s - T1 / r1s)*ynormal + d * (1 / r2s - 1 / r1s)*tnormal).matrix();
	}

};

}} //namespace