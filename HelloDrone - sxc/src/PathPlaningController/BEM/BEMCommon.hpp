#pragma once

//#include <boost/math/quadrature/gauss.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <vector>

#ifndef M_PI
#define M_PI static_cast<double>(3.1415926535897932384626433832795028841972)
#endif

namespace sxc { namespace bem{

using std::cout;
using std::endl;
using std::tuple;
using std::vector;
using std::map;
using std::pair;

//using boost::math::quadrature::gauss;

using Eigen::Vector2d;
using Eigen::Array2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::EigenSolver;
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

}} //namespace