#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic2d.h"
#include "Assert.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;

Hyperbolic2d::Point::Point(double x, double y) {
	assert(y > 0.00000001);
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
	return this->getGeodesic(point)->getVector();
}

Vector2d Hyperbolic2d::Line::getVector() {
	return z;
}

Vector2d Hyperbolic2d::Circle::getVector() {
	return z;
}

std::tr1::shared_ptr<Hyperbolic2d::Geodesic> Hyperbolic2d::Point::getGeodesic(Hyperbolic2d::Point point) {
	std::tr1::array<double,2> x = this->getCoordinates();
	std::tr1::array<double,2> y = point.getCoordinates();
	assert(x[1] > 0);
	if(fabs(x[0] - y[0]) < 0.00001) {
		return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Line(this, point, Vector2d(0.,log(y[1]/x[1]))));
	}
	//std::cout << "fabs(x[0] - y[0]) >= 0.00001\n";
	double c = ((x[0]*x[0]+x[1]*x[1])-(y[0]*y[0]+y[1]*y[1]))/(2*(x[0]-y[0]));
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dir0 = x[1]/r;
	double dir1 = (c-x[0])/r;
	double dist = log((x[1]*(r+y[0]-c))/(y[1]*(r+x[0]-c)));
	Vector2d z(dir0*dist,dir1*dist);
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
	assert(z[0] == z[0]);
	assert(z[1] == z[1]);
	/*std::cout << "pointFromVector(out).coordinates[0]:	" << pointFromVector(out).coordinates[0] << "\n";
	std::cout << "pointFromVector(out).coordinates[1]:	" << pointFromVector(out).coordinates[1] << "\n";
	std::cout << "point.coordinates[0]:	" << point.coordinates[0] << "\n";
	std::cout << "point.coordinates[1]:	" << point.coordinates[1] << "\n";*/
	assert(fabs(this->pointFromVector(z).getCoordinates()[0] - point.getCoordinates()[0]) < 0.000001);
	assert(fabs(this->pointFromVector(z).getCoordinates()[1] - point.getCoordinates()[1]) < 0.000001);
	return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Circle(this, point, z, c, r));
}

Hyperbolic2d::Point Hyperbolic2d::Point::pointFromVector(Vector2d z) {
	return this->getGeodesic(z)->getEndPoint();
}

std::tr1::shared_ptr<Hyperbolic2d::Geodesic> Hyperbolic2d::Point::getGeodesic(Vector2d z) {
	assert(z == z);
	std::tr1::array<double,2> x = this->getCoordinates();
	assert(x[1] > 0.00000001);
	if(fabs(z[0]) < 0.000001) {
		return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Line(this, Hyperbolic2d::Point(x[0],x[1]*exp(z[1])), z));
	}
	//std::cout << "fabs(z[0]) >= 0.000001\n";
	double c = x[0]+x[1]*z[1]/z[0];
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dist = sqrt(z[0]*z[0]+z[1]*z[1]);
	if(z[0] > 0) {
		dist *= -1;
		//std::cout << "z[0] > 0\n";
	} else {
		//std::cout << "z[0] <= 0\n";
	}
	double rx0c2 = r+x[0]-c;
	rx0c2 *= rx0c2;
	double x12e2d = x[1]*x[1]*exp(2*dist);
	double y0 = ((r+c)*rx0c2-x12e2d*(r-c))/(x12e2d+rx0c2);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	assert(y1 > 0.00000001);
	//std::cout << "2d vector:	(" << z[0] << ",	" << z[1] << ")\n";
	//std::cout << "2d point:	(" << y0 << ",	" << y1 << ")\n";
	/*std::cout << "z:\n" << z << "\n";
	std::cout << "c:	" << c << "\n";
	std::cout << "r:	" << r << "\n";*/
	//assert(fabs(log((y1*(r+x[0]-c))/(x[1]*(r+y0-c))) - dist) < 0.000001);
	Hyperbolic2d::Point end(y0,y1);
	return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Circle(this, end, z, c, r));
}

Hyperbolic2d::Point Hyperbolic2d::Line::getStartPoint() {
	return start;
}

Hyperbolic2d::Point Hyperbolic2d::Circle::getStartPoint() {
	return start;
}

Hyperbolic2d::Point Hyperbolic2d::Line::getEndPoint() {
	return end;
}

Hyperbolic2d::Point Hyperbolic2d::Circle::getEndPoint() {
	return end;
}

std::pair<Hyperbolic2d::Point, double> Hyperbolic2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic = this->getGeodesic(z);
	return std::pair<Hyperbolic2d::Point, double>(geodesic->getEndPoint(), geodesic->getRot());
}

double Hyperbolic2d::Line::getRot() {
	return 0;
}

double Hyperbolic2d::Circle::getRot() {
	std::tr1::array<double,2> x = start.getCoordinates();
	std::tr1::array<double,2> y = end.getCoordinates();
	double angle0 = atan2(x[1],x[0]-c);	//This is pointing towards the center. Since I'm just looking at the difference, it doesn't really matter.
	double angle1 = atan2(y[1],y[0]-c);
	//std::cout << "2d:" << std::endl;
	//std::cout << "x[1]:	" << x[1] << std::endl;
	//std::cout << "x[0]-c:	" << x[0]-c << "\nangle0:	" << angle0 << "\nangle1:	" << angle1 << std::endl;
	//This could be optimized by taking the arccos of the dot product.
	return angle1-angle0;
}

Hyperbolic2d::Circle::Circle(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z, double c, double r) {
	this->start = *start;
	this->end = end;
	this->c = c;
	this->r = r;
	this->z = z;
}

Hyperbolic2d::Line::Line(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z) {
	this->start = *start;
	this->end = end;
	this->z = z;
	assert(fabs(z[0]) < EPSILON);
}

