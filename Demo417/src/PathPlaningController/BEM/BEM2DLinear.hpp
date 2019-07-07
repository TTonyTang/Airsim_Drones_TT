#pragma once

#include "BEMCommon.hpp"
#include "BEM2D.hpp"
#include "BEMMathUtil.hpp"

namespace sxc { namespace bem {

struct LinearElement : Element {
	static const int nodesNumPerEle = 1;
	Vector2d p1;
	Vector2d p2;
	Vector2d normal;
	Vector2d unit;
	double u1 = 0.0;
	double u2 = 0.0;
	double q1 = 0.0;
	double q2 = 0.0;
	double length = 0.0;
	LinearElement(const vector<Vector2d>& points, int i) {
		p1 = points[i];
		p2 = points[(i + 1) % points.size()];
		const Vector2d delta = p2 - p1;
		length = delta.norm();
		unit = delta / this->length;
		const Vector2d perp(delta.y(), -delta.x());
		normal = perp / this->length;
	}
	double pointOnEle(const Vector2d& x) const override {
		if (BEMMathUtil::isPointOnLine(p1, p2, x)) {
			return 0;
		}
		else {
			return -2;
		}
	}
	Vector2d getPoint(const double t) const override {
		return ((1 - t)*p1 + (1 + t)*p2) / 2;
	}
	Vector2d getN(const double t) const override {
		return normal;
	}
	double getU(const double t) const override {
		return ((1 - t)*u1 + (1 + t)*u2) / 2;
	}
	double getQ(const double t) const override {
		return ((1 - t)*q1 + (1 + t)*q2) / 2;
	}
};

class BEM2DLinear : public BEM2D<LinearElement> {

public:

	void solve() override {
		const int N = this->nodesNum;
		MatrixXd G(this->nodesNum, this->nodesNum);
		for (auto i = 0; i < this->nodesNum; i++) {
			for (auto j = 0; j < this->nodesNum; j++) {
				if (i == j) {
					const auto& elej = this->elements[j];
					G(i, j) = elej.length *(1 - log(elej.length / 2));
				}
				else {
					G(i, j) = Gxj1(this->elements[i].p1, this->elements[j])+ Gxj2(this->elements[i].p1, this->elements[(j-1+N)%N]);
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
					H(i, j) = Hxj1(this->elements[i].p1, this->elements[j])+ Hxj2(this->elements[i].p1, this->elements[(j-1+N) % N]);
				}
			}
		}
		std::tie(this->baseUs, this->baseQs) = solveLinearAlgebra(H, G, this->baseUs, this->baseQs, this->mask);
		for (auto i = 0; i < this->nodesNum; i++) {
			this->elements[i].u1 = this->baseUs[i];
			this->elements[i].u2 = this->baseUs[(i + 1) % N];
			this->elements[i].q1 = this->baseQs[i];
			this->elements[i].q2 = this->baseQs[(i + 1) % N];
		}
	}

protected:

	static double Gxj1(const Vector2d& x, const LinearElement& elej) {
		const auto g = [x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto u = -log(r);
			return u * (1 - t)* elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(g);
	}

	static double Gxj2(const Vector2d& x, const LinearElement& elej) {
		const auto g = [x, elej](const double& t) {
			const auto y = elej.getPoint(t);
			const auto r = (x - y).norm();
			const auto u = -log(r);
			return u * (1 + t)* elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(g);
	}

	static double Hxj1(const Vector2d& x, const LinearElement& elej) {
		const double D = elej.normal.dot(elej.p1 - x);
		const auto h = [x, elej, D](const double& t) {
			const auto y = elej.getPoint(t);
			const double r = (x - y).norm();
			return -D * (1 - t) / (r * r) * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(h);
	}

	static double Hxj2(const Vector2d& x, const LinearElement& elej) {
		const double D = elej.normal.dot(elej.p1 - x);
		const auto h = [x, elej, D](const double& t) {
			const auto y = elej.getPoint(t);
			const double r = (x - y).norm();
			return -D * (1 + t) / (r * r) * elej.length / 2;
		};
		return BEMMathUtil::unitIntegrate(h);
	}

};

}} //namespace