#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "PortalSpace2d2.h"
#include "Hyperbolic2d.h"
#include "Assert.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;
using Eigen::Matrix2d;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

PortalSpace2d::Point::Point() {
}

PortalSpace2d::Point::Point(double t, double thetak) : hyperbolic(exp(thetak)/sqrt(1/(t*t)+1)*sgn(t),exp(thetak)/sqrt(t*t+1)) {
	assert(fabs(t - getT()) < EPSILON);
	assert(fabs(thetak - getThetaK()) < EPSILON);
}

PortalSpace2d::Point::Point(Hyperbolic2d::Point point) : hyperbolic(point.getCoordinates()[0],point.getCoordinates()[1]) {
}
//Hyperbolic::Point::~Point() {}

double PortalSpace2d::Point::getT(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return cartesian[0]/cartesian[1];
}

double PortalSpace2d::Point::getSin(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return cartesian[1]/sqrt(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1]);
}

double PortalSpace2d::Point::getCos(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return cartesian[0]/sqrt(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1]);
}

double PortalSpace2d::Point::getThetaK(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	return log(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1])/2;
}

std::tr1::array<double,2> PortalSpace2d::Point::getCoordinates(){
	std::tr1::array<double,2> cartesian = hyperbolic.getCoordinates();
	std::tr1::array<double,2> out;
	out[0] = cartesian[0]/cartesian[1];
	out[1] = log(cartesian[0]*cartesian[0]+cartesian[1]*cartesian[1])/2;
	return out;
}

Vector2d PortalSpace2d::Point::vectorFromPoint(PortalSpace2d::Point point) {
	Vector2d cartesian = hyperbolic.vectorFromPoint(point.hyperbolic);
	Matrix2d m;
	m <<	-getSin(),	getCos(),		//This matrix is the inverse of the other two. It turns out that it's its own inverse, so it's the same matrix.
			getCos(),	getSin();
	Vector2d polar = m*cartesian;
	/*std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[0]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[0] << "\n";
	std::cout << "point.hyperbolic.getCoordinates()[0]:	" << point.hyperbolic.getCoordinates()[0] << "\n";
	std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[1]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[1] << "\n";
	std::cout << "point.hyperbolic.getCoordinates()[1]:	" << point.hyperbolic.getCoordinates()[1] << "\n";
	//std::cout << "pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]:	" << pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0] << "\n";*/
	//std::cout << "fabs(pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]):	" << fabs(pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]) << "\n";
	//assert(fabs(pointFromVector(polar).hyperbolic.getCoordinates()[0] - point.hyperbolic.getCoordinates()[0]) < 0.00001);
	//This assertion tends to get cut close.
	//assert(fabs(pointFromVector(polar).hyperbolic.getCoordinates()[1] - point.hyperbolic.getCoordinates()[1]) < 0.00001);
	return polar;
}

PortalSpace2d::Point PortalSpace2d::Point::pointFromVector(Vector2d z) {
	assert(z == z);
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	Matrix2d m;
	m <<	-getSin(),	getCos(),
			getCos(),	getSin();
	Vector2d cartesian = m*z;
	assert(cartesian == cartesian);
	/*std::cout << "xangle:\n" << xangle << "\n";
	std::cout << "m:\n" << m << "\n";
	std::cout << "cartesian:\n" << cartesian << "\n";*/
	return PortalSpace2d::Point(hyperbolic.pointFromVector(cartesian));
}

void PortalSpace2d::Point::setCoordinates(double t, double thetak) {
	hyperbolic.setCoordinates(exp(thetak)/sqrt(1/(t*t)+1)*sgn(t),exp(thetak)/sqrt(t*t+1));
}

std::pair<PortalSpace2d::Point, double> PortalSpace2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	assert(coordinates[1] > 0);
	Vector2d xangle(coordinates[0],coordinates[1]);
	//std::cout << "position:\n" << xangle << "\n";
	xangle.normalize();
	Matrix2d m;
	m <<	-getSin(),	getCos(),
			getCos(),	getSin();
	Vector2d cartesian = m*z;
	std::pair<Hyperbolic2d::Point, double> pointAndRot = hyperbolic.pointAndRotFromVector(cartesian);
	assert(hyperbolic.pointFromVector(cartesian).getCoordinates()[1] > 0);
	assert(pointAndRot.first.getCoordinates()[1] > 0);
	//assert(pointAndRot.first == hyperbolic.pointFromVector(cartesian));
	PortalSpace2d::Point ypoint(pointAndRot.first);
	double rot = -pointAndRot.second - (atan(ypoint.getT()) - atan(getT()));
	//If I do rot like this, it's clearly broken. If I do rot the other way, the 3d extension is clearly broken.
	/*std::cout << "pointAndRot.first:	(" << pointAndRot.first.getCoordinates()[0] << ",	" << pointAndRot.first.getCoordinates()[1] << ")\n";
	std::cout << "pointAndRot.second:	" << pointAndRot.second << "\n";
	std::cout << "ypoint.getT():	" << ypoint.getT() << "\n";
	std::cout << "getT():	" << getT() << "\n";
	std::cout << "rot:	" << rot << "\n";*/
	/*std:: cout << "Hyperbolic2d rot:	" << pointAndRot.second << "\n";
	std:: cout << "Hyperbolic2d vector:\n" << cartesian << "\n";
	std:: cout << "PortalSpace2d rot:	" << rot << "\n";
	std:: cout << "PortalSpace2d vector:\n" << z << "\n";*/
	return std::pair<PortalSpace2d::Point, double>(ypoint,rot);
}

