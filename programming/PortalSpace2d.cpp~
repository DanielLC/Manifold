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
	return this->getGeodesic(point)->getVector();
}

PortalSpace2d::Point PortalSpace2d::Point::pointFromVector(Vector2d z) {
	return this->getGeodesic(z)->getEndPoint();
}

void PortalSpace2d::Point::setCoordinates(double x, double y) {
	hyperbolic.setCoordinates(x,y);
}

std::pair<PortalSpace2d::Point, double> PortalSpace2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::shared_ptr<PortalSpace2d::Geodesic> geodesic = this->getGeodesic(z);
	return std::pair<PortalSpace2d::Point, double>(geodesic->getEndPoint(), geodesic->getRot());
}

Hyperbolic2d::Point PortalSpace2d::Point::getPosition() {
	return hyperbolic;
}

std::tr1::shared_ptr<PortalSpace2d::Geodesic> PortalSpace2d::Point::getGeodesic(PortalSpace2d::Point point) {
	return std::tr1::shared_ptr<PortalSpace2d::Geodesic>(new PortalSpace2d::Geodesic(hyperbolic.getGeodesic(point.getPosition())));
}

std::tr1::shared_ptr<PortalSpace2d::Geodesic> PortalSpace2d::Point::getGeodesic(Vector2d z) {
	std::tr1::array<double,2> coordinates = hyperbolic.getCoordinates();
	double theta = getT();
	Matrix2d m;
	m <<	sin(theta),	cos(theta),
			cos(theta),	-sin(theta);
	Vector2d cartesian = m*z;
	return std::tr1::shared_ptr<PortalSpace2d::Geodesic>(new PortalSpace2d::Geodesic(hyperbolic.getGeodesic(cartesian)));
}

PortalSpace2d::Geodesic::Geodesic(std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic) {
	this->geodesic = geodesic;
}

PortalSpace2d::Point PortalSpace2d::Geodesic::getStartPoint() {
	return PortalSpace2d::Point(geodesic->getStartPoint());
}

PortalSpace2d::Point PortalSpace2d::Geodesic::getEndPoint() {
	return PortalSpace2d::Point(geodesic->getEndPoint());
}

double PortalSpace2d::Geodesic::getRot() {
	return -geodesic->getRot() - (this->getEndPoint().getT() - this->getStartPoint().getT());
}

double PortalSpace2d::Point::operator[](int i) {
	if(i == 0) {
		return getT();
	} else {
		return getThetaK();
	}
}

Vector2d PortalSpace2d::Geodesic::getVector() {
	Vector2d cartesian = geodesic->getVector();
	double theta = getStartPoint().getT();
	Matrix2d m;
	m <<	sin(theta),	cos(theta),		//This matrix is the inverse of the other two. It turns out that it's its own inverse, so it's the same matrix.
			cos(theta),	-sin(theta);
	Vector2d polar = m*cartesian;
	return polar;
}

