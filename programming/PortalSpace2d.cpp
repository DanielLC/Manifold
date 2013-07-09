#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "PortalSpace2d.h"
#include "Hyperbolic2d.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;
using Eigen::Matrix2d;

PortalSpace2d::Point::Point() {
}

PortalSpace2d::Point::Point(double t, double thetak) : hyperbolic(exp(thetak)*cos(t),exp(thetak)*sin(t)) {
}

PortalSpace2d::Point::Point(Hyperbolic2d::Point point) : hyperbolic(point.getCoordinates()[0],point.getCoordinates()[1]) {
}
//Hyperbolic::Point::~Point() {}

double PortalSpace2d::Point::getT(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return atan2(cartesian[1],cartesian[0]);
}

double PortalSpace2d::Point::getThetaK(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return sqrt(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1]);
}

std::tr1::array<double,2> PortalSpace2d::Point::getCoordinates(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	std::tr1::array<double,2> out;
	out[0] = atan2(cartesian[1],cartesian[0]);
	out[1] = sqrt(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1]);
	return out;
}

Vector2d PortalSpace2d::Point::vectorFromPoint(PortalSpace2d::Point point) {
	Vector2d cartesian = hyperbolic.vectorFromPoint(point.hyperbolic);
	double theta = getT();
	Matrix2d m;
	m <<	sin(theta),	cos(theta),		//This matrix is the inverse of the other two. It turns out that it's its own inverse, so it's the same matrix.
			cos(theta),	-sin(theta);
	Vector2d polar = m*cartesian;
	/*std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[0]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[0] << "\n";
	std::cout << "point.hyperbolic.getCoordinates()[0]:	" << point.hyperbolic.getCoordinates()[0] << "\n";
	std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[1]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[1] << "\n";
	std::cout << "point.hyperbolic.getCoordinates()[1]:	" << point.hyperbolic.getCoordinates()[1] << "\n";*/
	//std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0] << "\n";
	/*assert(fabs(pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]) < 0.000001);
	assert(fabs(pointFromVector(polar).hyperbolic.getCoordinates()[1] - point.hyperbolic.getCoordinates()[1]) < 0.000001);*/
	//These assertions occasionally fail. There doesn't seem to be any sort of growth in error. It's just occationally huge.
	return polar;
}

PortalSpace2d::Point PortalSpace2d::Point::pointFromVector(Vector2d z) {
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	double theta = getT();
	Matrix2d m;
	m <<	sin(theta),	cos(theta),
			cos(theta),	-sin(theta);
	Vector2d cartesian = m*z;
	/*std::cout << "xangle:\n" << xangle << "\n";
	std::cout << "m:\n" << m << "\n";
	std::cout << "cartesian:\n" << cartesian << "\n";*/
	return PortalSpace2d::Point(hyperbolic.pointFromVector(cartesian));
}

void PortalSpace2d::Point::setCoordinates(double x, double y) {
	hyperbolic.setCoordinates(x,y);
}

std::pair<PortalSpace2d::Point, double> PortalSpace2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	assert(coordinates[1] > 0);
	Vector2d xangle(coordinates[0],coordinates[1]);
	//std::cout << "position:\n" << xangle << "\n";
	xangle.normalize();
	double theta = getT();
	Matrix2d m;
	m <<	sin(theta),	cos(theta),
			cos(theta),	-sin(theta);
	Vector2d cartesian = m*z;
	std::pair<Hyperbolic2d::Point, double> pointAndRot = hyperbolic.pointAndRotFromVector(cartesian);
	assert(hyperbolic.pointFromVector(cartesian).getCoordinates()[1] > 0);
	assert(pointAndRot.first.getCoordinates()[1] > 0);
	//assert(pointAndRot.first == hyperbolic.pointFromVector(cartesian));
	PortalSpace2d::Point ypoint(pointAndRot.first);
	double rot = -pointAndRot.second - (ypoint.getT() - getT());
	/*std::cout << "pointAndRot.first:	(" << pointAndRot.first.getCoordinates()[0] << ",	" << pointAndRot.first.getCoordinates()[1] << ")\n";
	std::cout << "pointAndRot.second:	" << pointAndRot.second << "\n";
	std::cout << "ypoint.getT():	" << ypoint.getT() << "\n";
	std::cout << "getT():	" << getT() << "\n";
	std::cout << "rot:	" << rot << "\n";*/
	return std::pair<PortalSpace2d::Point, double>(ypoint,rot);
}

