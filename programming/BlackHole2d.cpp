#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "BlackHole2d.h"
#include "Hyperbolic2d.h"
#include "Assert.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;
using Eigen::Matrix2d;

BlackHole2d::Point::Point() {
}

BlackHole2d::Point::Point(double t, double thetak) : hyperbolic(thetak, t) {
}

BlackHole2d::Point::Point(Hyperbolic2d::Point point) : hyperbolic(point.getCoordinates()[0],point.getCoordinates()[1]) {
}

double BlackHole2d::Point::getT(){
	return hyperbolic.getCoordinates()[1];
}

double BlackHole2d::Point::getThetaK(){
	return hyperbolic.getCoordinates()[0];
}

std::tr1::array<double,2> BlackHole2d::Point::getCoordinates(){
	std::tr1::array<double,2> x = hyperbolic.getCoordinates();
	std::tr1::array<double,2> out;
	out[0] = x[1];
	out[1] = x[0];
	return out;
}

Vector2d BlackHole2d::Point::vectorFromPoint(BlackHole2d::Point point) {
	Vector2d in = hyperbolic.vectorFromPoint(point.hyperbolic);
	Vector2d out = (Matrix2d() << 0,1,1,0).finished()*in;
	assert(fabs(pointFromVector(out).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]) < 0.00001);
	assert(fabs(pointFromVector(out).hyperbolic.getCoordinates()[1] - point.hyperbolic.getCoordinates()[1]) < 0.00001);
	return out;
}

BlackHole2d::Point BlackHole2d::Point::pointFromVector(Vector2d z) {
	assert(z == z);
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	Matrix2d m;
	m <<	0,	1,
			1,	0;
	Vector2d out = m*z;
	assert(out == out);
	return BlackHole2d::Point(hyperbolic.pointFromVector(out));
}

void BlackHole2d::Point::setCoordinates(double t, double thetak) {
	hyperbolic.setCoordinates(thetak, t);
}

std::pair<BlackHole2d::Point, double> BlackHole2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	assert(coordinates[1] > 0);
	Vector2d xangle(coordinates[0],coordinates[1]);
	//std::cout << "position:\n" << xangle << "\n";
	xangle.normalize();
	Matrix2d m;
	m <<	0,	1,
			1,	0;
	Vector2d cartesian = m*z;
	std::pair<Hyperbolic2d::Point, double> pointAndRot = hyperbolic.pointAndRotFromVector(cartesian);
	assert(hyperbolic.pointFromVector(cartesian).getCoordinates()[1] > 0);
	assert(pointAndRot.first.getCoordinates()[1] > 0);
	BlackHole2d::Point ypoint(pointAndRot.first);
	double rot = -pointAndRot.second;
	/*std::cout << "pointAndRot.first:	(" << pointAndRot.first.getCoordinates()[0] << ",	" << pointAndRot.first.getCoordinates()[1] << ")\n";
	std::cout << "pointAndRot.second:	" << pointAndRot.second << "\n";
	std::cout << "ypoint.getT():	" << ypoint.getT() << "\n";
	std::cout << "getT():	" << getT() << "\n";
	std::cout << "rot:	" << rot << "\n";*/
	return std::pair<BlackHole2d::Point, double>(ypoint,rot);
}

