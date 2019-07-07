#pragma once

#include <chrono>
#include <thread>
#include <iostream>
#include <limits>
#include <type_traits>
#include <iomanip>

namespace sxc { namespace path_planning_controller {

using std::cout;
using std::endl;

using clk = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clk>;
using duration = std::chrono::duration<double>;

static std::clock_t c_start;
static std::clock_t c_end;
static time_point t_start;
static time_point t_end;

inline void sleep(int milliseconds) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

inline void tic() {
	c_start = std::clock();
	t_start = std::chrono::high_resolution_clock::now();
}

inline void toc() {
	c_end = std::clock();
	t_end = std::chrono::high_resolution_clock::now();
	cout << std::fixed << std::setprecision(2) << "CPU time used: "
		<< 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n"
		<< "Wall clock time passed: "
		<< std::chrono::duration<double, std::milli>(t_end - t_start).count()
		<< " ms\n";
}

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