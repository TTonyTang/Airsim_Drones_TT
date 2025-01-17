// BEMPlot.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Eigen"
#include "PathPlaningController/BEM/BEM2DConstant.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign.hpp>

#include <io.h>
#include <direct.h>


using BEM = sxc::bem::BEM2DConstantAnalytical;
using DPoint = boost::geometry::model::d2::point_xy<double>;
using DPolygon = boost::geometry::model::polygon<DPoint, false>;

void test(const std::string root, const int caseNo) {
	sxc::bem::BEM2DConstantAnalytical bemcase{};

	const std::string no = std::to_string(caseNo);

	std::ifstream inmesh(root+"data/mesh-" + no + ".txt");
	std::vector<Eigen::Vector2d> mesh{};
	double x, y;
	double xmin = std::numeric_limits<double>::max();
	double xmax = std::numeric_limits<double>::min();
	double ymin = std::numeric_limits<double>::max();
	double ymax = std::numeric_limits<double>::min();
	while (inmesh.peek() != EOF)
	{
		inmesh >> x >> y;
		xmin = std::min(xmin, x);
		xmax = std::max(xmax, x);
		ymin = std::min(ymin, y);
		ymax = std::max(ymax, y);
		//cout << x << "," << y << endl;
		mesh.emplace_back(Eigen::Vector2d(x, y));
		inmesh.get();
	}
	inmesh.close();

	std::map<int, double> bc1{};
	std::map<int, double> bc2{};
	int index;
	double value;

	std::ifstream incondition1(root+"data/condition1-" + no + ".txt");
	while (incondition1.peek() != EOF)
	{
		incondition1 >> index >> value;
		bc1[index] = value;
		incondition1.get();
	}
	incondition1.close();

	std::ifstream incondition2(root+"data/condition2-" + no + ".txt");
	while (incondition2.peek() != EOF)
	{
		incondition2 >> index >> value;
		bc2[index] = value;
		incondition2.get();
	}
	incondition2.close();

	bemcase.setMesh(mesh);
	bemcase.setBoundaryCondition(bc1, bc2);
	bemcase.solve();

	std::ofstream outx(root + "data/plot data/x-" + no + ".txt");
	std::ofstream outy(root + "data/plot data/y-" + no + ".txt");
	std::ofstream outz(root + "data/plot data/z-" + no + ".txt");
	outx.precision(20);
	outy.precision(20);
	outz.precision(20);

	DPolygon poly;
	std::vector<DPoint> list;
	for (const auto p : mesh) {
		list.emplace_back(DPoint(p.x(), p.y()));
	}
	list.emplace_back(DPoint(mesh[0].x(),mesh[0].y()));
	poly.outer().assign(list.begin(), list.end());

	const int N = 200;
	std::vector<double> xs(N + 1), ys(N + 1);

	for (int i = 0; i <= N; i++) {
		xs[i] = xmin + (xmax - xmin)*i / N;
		ys[i] = ymin + (ymax - ymin)*i / N;
	}
	for (int j = 0; j <= N; j++) {
		for (int i = 0; i <= N; i++) {
			//std::cout << X[i] << "," << Y[j] << std::endl;
			outx << xs[i] << std::endl;
			outy << ys[j] << std::endl;
			const DPoint Pts(xs[i], ys[j]);
			const Eigen::Vector2d point(xs[i], ys[j]);
			if(boost::geometry::within(Pts, poly)){
				//std::cout << "Inside" << std::endl;
				const auto ret = bemcase.getInnerPotentialAndFlux(point);
				double p;
				Eigen::Vector2d flux;
				std::tie(p, flux) = ret;
				outz << std::min(p, 100.0) << " " << flux.x() << " " << flux.y() << std::endl;
			}else{
				outz << "nan nan nan" << std::endl;
			}
		}
	}
	outx.close();
	outy.close();
	outz.close();
}

void test2(const std::string filename) {
  sxc::bem::BEM2DConstantAnalytical bemcase{};

  const std::string originPath = "origin/";
  const std::string dataPath = "data/";

  std::ifstream inmesh(originPath + filename + "-mesh.txt");
  std::vector<Eigen::Vector2d> mesh{};
  double x, y;
  double xmin = std::numeric_limits<double>::max();
  double xmax = std::numeric_limits<double>::min();
  double ymin = std::numeric_limits<double>::max();
  double ymax = std::numeric_limits<double>::min();
  while (inmesh.peek() != EOF) {
    inmesh >> x >> y;
    xmin = std::min(xmin, x);
    xmax = std::max(xmax, x);
    ymin = std::min(ymin, y);
    ymax = std::max(ymax, y);
    // cout << x << "," << y << endl;
    mesh.emplace_back(Eigen::Vector2d(x, y));
    inmesh.get();
  }
  inmesh.close();

	std::cout << mesh.size() << std::endl;

  std::map<int, double> bc1{};
  std::map<int, double> bc2{};
  int index;
  double value;

  std::ifstream incondition1(originPath + filename + "-condition1.txt");
  while (incondition1.peek() != EOF) {
    incondition1 >> index >> value;
    bc1[index] = value;
    incondition1.get();
  }
  incondition1.close();

  std::ifstream incondition2(originPath + filename + "-condition2.txt");
  while (incondition2.peek() != EOF) {
    incondition2 >> index >> value;
    bc2[index] = value;
    incondition2.get();
  }
  incondition2.close();

  bemcase.setMesh(mesh);
  bemcase.setBoundaryCondition(bc1, bc2);
  bemcase.solve();

	
  if (_access(dataPath.c_str(), 0) == -1) {
    int flag = _mkdir(dataPath.c_str());
  }
  std::ofstream outx(dataPath + filename + "-x.txt");
  std::ofstream outy(dataPath + filename + "-y.txt");
  std::ofstream outz(dataPath + filename + "-z.txt");
  outx.precision(20);
  outy.precision(20);
  outz.precision(20);

  DPolygon poly;
  std::vector<DPoint> list;
  for (const auto p : mesh) {
    list.emplace_back(DPoint(p.x(), p.y()));
  }
  list.emplace_back(DPoint(mesh[0].x(), mesh[0].y()));
  poly.outer().assign(list.begin(), list.end());

  const int N = 200;
  std::vector<double> xs(N), ys(N);

  for (int i = 0; i < N; i++) {
    xs[i] = xmin + (xmax - xmin) * i / (N-1);
    ys[i] = ymin + (ymax - ymin) * i / (N-1);
  }
  for (int j = 0; j < N; j++) {
    for (int i = 0; i < N; i++) {
      // std::cout << X[i] << "," << Y[j] << std::endl;
      outx << xs[i] << std::endl;
      outy << ys[j] << std::endl;
      const DPoint Pts(xs[i], ys[j]);
      const Eigen::Vector2d point(xs[i], ys[j]);
      if (boost::geometry::within(Pts, poly)) {
        // std::cout << "Inside" << std::endl;
        const auto ret = bemcase.getInnerPotentialAndFlux(point);
        double p;
        Eigen::Vector2d flux;
        std::tie(p, flux) = ret;
        outz << std::min(p, 100.0) << " " << flux.x() << " " << flux.y()
             << std::endl;
      } else {
        outz << "nan nan nan" << std::endl;
      }
    }
  }
  outx.close();
  outy.close();
  outz.close();
}

int main()
{
	//const std::string root = "../HelloDrone - Copy/";
	const std::string root = "S:/AirSim/SingleDrone/";
	std::ifstream in("list.txt");
	int no;
	std::vector<int> nos;
	while (in.peek() != EOF)
	{
		in >> no;
		nos.emplace_back(no);
		in.get();
	}
	//test(root, 1);
	for(const int i: nos){
		std::cout << i << std::endl;
		test(root, i);
	}
	
	//test2("test");
	system("pause");
    return 0;
}

