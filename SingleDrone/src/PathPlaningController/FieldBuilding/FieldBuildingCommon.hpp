#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <map>

#ifndef M_PI
#define M_PI static_cast<double>(3.1415926535897932384626433832795028841972)
#endif

namespace sxc { namespace field_building {

using std::cout;
using std::endl;
using std::tuple;
using std::vector;
using std::map;
using std::pair;

using Eigen::Vector2d;

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
	almost_equal(T x, T y, int ulp) {
	// the machine epsilon has to be scaled to the magnitude of the values used
	// and multiplied by the desired precision in ULPs (units in the last place)
	return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
		// unless the result is subnormal
		|| std::abs(x - y) < std::numeric_limits<T>::min();
}

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
	almost_zero(T x, int ulp) {
	return std::abs(x) <= std::numeric_limits<T>::epsilon() * ulp || std::abs(x) < std::numeric_limits<T>::min();
}

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
	almost_equal(T x, T y) {
	return almost_equal(x, y, 10);
}

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
	almost_zero(T x) {
	return almost_zero(x, 10);
}

const vector<pair<Vector2d, Vector2d>> globalMap{
	pair<Vector2d, Vector2d> {Vector2d(-10,-10), Vector2d(-10,50)},
	pair<Vector2d, Vector2d> {Vector2d(-10,50), Vector2d(30,50)},
	pair<Vector2d, Vector2d> {Vector2d(30,50), Vector2d(30,70)},
	pair<Vector2d, Vector2d> {Vector2d(30,70), Vector2d(-30,70)},
	pair<Vector2d, Vector2d> {Vector2d(-30,70), Vector2d(-30,-10)},
	pair<Vector2d, Vector2d> {Vector2d(-30,-10), Vector2d(-10,-10)},

	pair<Vector2d, Vector2d> {Vector2d(10,-10), Vector2d(10,30)},
	pair<Vector2d, Vector2d> {Vector2d(10,30), Vector2d(50,30)},
	pair<Vector2d, Vector2d> {Vector2d(50,30), Vector2d(50,10)},
	pair<Vector2d, Vector2d> {Vector2d(50,10), Vector2d(30,10)},
	pair<Vector2d, Vector2d> {Vector2d(30,10), Vector2d(30,-10)},
	pair<Vector2d, Vector2d> {Vector2d(30,-10), Vector2d(10,-10)},
};

}} //namespace