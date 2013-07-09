#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic2d.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;

Hyperbolic2d::Point::Point(double x, double y) {
	coordinates[0] = x;
	coordinates[1] = y;
}

Hyperbolic2d::Point::Point() {
	coordinates[1] = 1;
}

std::tr1::array<double,2> Hyperbolic2d::Point::getCoordinates(){return coordinates;}

void Hyperbolic2d::Point::setCoordinates(double x, double y) {
	coordinates[0] = x;
	coordinates[1] = y;
}

Vector2d Hyperbolic2d::Point::vectorFromPoint(Hyperbolic2d::Point point) {
	std::tr1::array<double,2> x = this->coordinates;
	std::tr1::array<double,2> y = point.coordinates;
	assert(x[1] > 0);
	if(fabs(x[0] - y[0]) < 0.00001) {
		return Vector2d(0.,log(y[1]/x[1]));
	}
	double c = ((x[0]*x[0]+x[1]*x[1])-(y[0]*y[0]+y[1]*y[1]))/(2*(x[0]-y[0]));
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dir0 = x[1]/r;
	double dir1 = (c-x[0])/r;
	double dist = log((x[1]*(r+y[0]-c))/(y[1]*(r+x[0]-c)));
	Vector2d out(dir0*dist,dir1*dist);
	/*std::cout << "y:	(" << y[0] << ",	" << y[1] << ")\n";
	std::cout << "c:	" << c << "\n";
	std::cout << "r:	" << r << "\n";
	std::cout << "dir0:	" << dir0 << "\n";
	std::cout << "dir1:	" << dir1 << "\n";
	std::cout << "dist:	" << dist << "\n";
	std::cout << "(r+x[0]-c):	" << (r+x[0]-c) << "\n";
	std::cout << "(r+y[0]-c):	" << (r+y[0]-c) << "\n";
	std::cout << "x[1]:	" << x[1] << "\n";
	std::cout << "y[1]:	" << y[1] << "\n";
	std::cout << "(y[1]*(r+x[0]-c))/(x[1]*(r+y[0]-c)):	" << (y[1]*(r+x[0]-c))/(x[1]*(r+y[0]-c)) << "\n";*/
	assert(out[0] == out[0]);
	assert(out[1] == out[1]);
	/*assert(fabs(pointFromVector(out).coordinates[0] - point.coordinates[0]) < 0.000001);
	assert(fabs(pointFromVector(out).coordinates[1] - point.coordinates[1]) < 0.000001);*/
	return out;
}

Hyperbolic2d::Point Hyperbolic2d::Point::pointFromVector(Vector2d z) {
	assert(z == z);
	std::tr1::array<double,2> x = coordinates;
	if(fabs(z[0]) < 0.000001) {
		return Hyperbolic2d::Point(x[0],x[1]*exp(z[1]));
	}
	double c = x[0]+x[1]*z[1]/z[0];
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dist = sqrt(z[0]*z[0]+z[1]*z[1]);
	if(z[0] > 0) {
		dist *= -1;
	}
	double rx0c2 = r+x[0]-c;
	rx0c2 *= rx0c2;
	double x12e2d = x[1]*x[1]*exp(2*dist);
	double y0 = ((r+c)*rx0c2-x12e2d*(r-c))/(x12e2d+rx0c2);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	//std::cout << "2d vector:	(" << z[0] << ",	" << z[1] << ")\n";
	//std::cout << "2d point:	(" << y0 << ",	" << y1 << ")\n";
	/*std::cout << "z:\n" << z << "\n";
	std::cout << "c:	" << c << "\n";
	std::cout << "r:	" << r << "\n";*/
	//assert(fabs(log((y1*(r+x[0]-c))/(x[1]*(r+y0-c))) - dist) < 0.000001);
	assert(x[1] > 0.00000001);
	assert(y1 > 0.00000001);
	return Hyperbolic2d::Point(y0,y1);
}

std::pair<Hyperbolic2d::Point, double> Hyperbolic2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::array<double,2> x = coordinates;
	if(fabs(z[0]) < 0.000001) {
		return std::pair<Hyperbolic2d::Point, double>
			(Hyperbolic2d::Point(x[0],x[1]*exp(z[1])), 0);
	}
	double c = x[0]+x[1]*z[1]/z[0];
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dist = sqrt(z[0]*z[0]+z[1]*z[1]);
	if(z[0] > 0) {
		dist *= -1;
	}
	double rx0c2 = r+x[0]-c;
	rx0c2 *= rx0c2;
	double x12e2d = x[1]*x[1]*exp(2*dist);
	double y0 = ((r+c)*rx0c2-x12e2d*(r-c))/(x12e2d+rx0c2);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	
	//std::cout << "pointFromVector(z).coordinates[0]:	" << pointFromVector(z).coordinates[0] << "\n";
	//std::cout << "coordinates[0]:	" << coordinates[0] << "\n";
	//assert(pointFromVector(z).coordinates[0] == y0);
	//assert(pointFromVector(z).coordinates[1] == y1);
	//std::cout << "z:\n" << z << "\n";
	assert(y1 > 0.00000001);
	
	double angle0 = atan2(x[1],x[0]-c);	//This is pointing towards the center. Since I'm just looking at the difference, it doesn't really matter.
	double angle1 = atan2(y1,y0-c);
	//This could be optimized by taking the arccos of the dot product.
	return std::pair<Hyperbolic2d::Point, double>(Hyperbolic2d::Point(y0,y1), angle1-angle0);	//angle1-angle0 has no gaurantee of being in the 0 to 2*pi range, but since I'm always going to just take the sine and cosine, this won't matter.
}

