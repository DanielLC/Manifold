#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "PortalSpace2d2.h"
#include "Hyperbolic2d.h"
#include "SurfaceOfRevolution.h"
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
	//t is cot(theta).
	//std::cout << "t:\t" << t << "\ntheta k:\t" << thetak << std::endl;
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
	return this->getGeodesic(point)->getVector();
}

Vector2d PortalSpace2d::Geodesic::getVector() {
	//std::cout << "From:\t" << getStartPoint().getT() << ",\t" << getStartPoint().getThetaK() << std::endl;
	//std::cout << "To:\t" << getEndPoint().getT() << ",\t" << getEndPoint().getThetaK() << std::endl;
	Vector2d cartesian = geodesic->getVector();
	double sin = getStartPoint().getSin();
	double cos = getStartPoint().getCos();
	Matrix2d m;
	m <<	-sin,	cos,	//This matrix is the inverse of the other two. It turns out that it's its own inverse, so it's the same matrix.
			cos,	sin;
	Vector2d polar = m*cartesian;
	//std::cout << "Vector:\n" << polar << std::endl;
	return polar;
}

PortalSpace2d::Point PortalSpace2d::Point::pointFromVector(Vector2d z) {
	return this->getGeodesic(z)->getEndPoint();
}

std::tr1::shared_ptr<PortalSpace2d::Geodesic> PortalSpace2d::Point::getGeodesic(Vector2d z) {
	assert(z == z);
	double sin = getSin();
	double cos = getCos();
	Matrix2d m;
	m <<	-sin,	cos,	//This matrix is the inverse of the other two. It turns out that it's its own inverse, so it's the same matrix.
			cos,	sin;
	Vector2d cartesian = m*z;
	assert(cartesian == cartesian);
	return std::tr1::shared_ptr<PortalSpace2d::Geodesic>(new PortalSpace2d::Geodesic(hyperbolic.getGeodesic(cartesian)));
}

void PortalSpace2d::Point::setCoordinates(double t, double thetak) {
	hyperbolic.setCoordinates(exp(thetak)/sqrt(1/(t*t)+1)*sgn(t),exp(thetak)/sqrt(t*t+1));
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
	//std::cout << "x.t:\t" << getEndPoint().getT() << "\ny.t:\t" << getStartPoint().getT() << "\nhyperbolic rot:\t" << geodesic->getRot() << std::endl;
	double rot = -geodesic->getRot() - (atan(this->getEndPoint().getT()) - atan(this->getStartPoint().getT()));
	//std::cout << "rot:\t" << rot << std::endl;
	return rot;
}

double PortalSpace2d::Geodesic::intersectionDistance(double portal) {
	//std::cout << "PortalSpace2d2.cpp intersectionDistance" << std::endl;
	assert(geodesic);
	double dist = geodesic->wormholeIntersectionDistance(portal);
	assert(dist > 0);
	return dist;
}

Intersection2d PortalSpace2d::Geodesic::getIntersection(double portal) {
	return geodesic->wormholeGetIntersection(portal);
}

double PortalSpace2d::Point::operator[](int i) {
	if(i == 0) {
		return getT();
	} else {
		return getThetaK();
	}
}

/*static PortalSpace2d::GeodesicPtr PortalSpace2d::getGeodesic(Intersection2d intersection, double portal) {
	return PortalSpace2d::GeodesicPtr(new PortalSpace2d::Geodesic(Hyperbolic2d::wormholeGetGeodesic(intersection, portal)));
}*/

std::string PortalSpace2d::getType() {
	return "PortalSpace2d";
}

//For SurfaceOfRevolution<PortalSpace2d>

double getEDist(double cot) {	//I probably shouldn't make this global. I'm only using it in these next two methods.
	double csc = sqrt(cot*cot+1);
	return csc + cot;
}

template<>
PointTransportPtr SurfaceOfRevolution<PortalSpace2d>::Portal::getTransport(Manifold::PointPtr point) {
	SurfaceOfRevolution<PortalSpace2d>::Point* castedPoint = (SurfaceOfRevolution<PortalSpace2d>::Point*) point.get();
	//std::cout << "PortalSpace2d2.cpp Portal coordinate:\t" << t << std::endl;
	//std::cout << "PortalSpace2d2.cpp Point coordinate:\t" << castedPoint->getT() << std::endl;
	double dist = log(getEDist(castedPoint->getT())/getEDist(t));	//TODO: there's a chance this is backwards.
	if(getInvert()) {
		dist = -dist;
	}
	return PointTransportPtr(new PointTransport(castedPoint->getSpherical(), dist));
}

template<>
Manifold::PointPtr SurfaceOfRevolution<PortalSpace2d>::Portal::getPoint(PointTransportPtr transport) {
	Vector3d spherical = transport->getPosition();
	//std::cout << "PortalSpace2d2.cpp t:\t" << t << std::endl;
	double d = getEDist(t);
	if(getInvert()) {
		 d *= exp(transport->getDistance());
	} else {
		d *= exp(-transport->getDistance());
	}
	double tt = (d-1/d)/2;
	//std::cout << "PortalSpace2d2.cpp portal dist:\t" << log(getEDist(t)) << std::endl;
	//std::cout << "PortalSpace2d2.cpp distance:\t" << transport->getDistance() << std::endl;
	//std::cout << "PortalSpace2d2.cpp invert:\t" << getInvert() << std::endl;
	//std::cout << "PortalSpace2d2.cpp tt:\t" << tt << std::endl;
	Manifold::PointPtr out(new SurfaceOfRevolution<PortalSpace2d>::Point(tt, spherical[0], spherical[1], spherical[2], (SurfaceOfRevolution<PortalSpace2d>*) getSpace()));
	#ifndef NDEBUG
	SurfaceOfRevolution<PortalSpace2d>::PointPtr testPoint(new SurfaceOfRevolution<PortalSpace2d>::Point(t, spherical[0], spherical[1], spherical[2], (SurfaceOfRevolution<PortalSpace2d>*) getSpace()));
	Manifold::PointOfReferencePtr testPointOfReference(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(testPoint, Matrix4d::Identity()));
	SurfaceOfRevolution<PortalSpace2d>::PointPtr otherTestPoint(new SurfaceOfRevolution<PortalSpace2d>::Point(0, spherical[0], spherical[1], spherical[2], (SurfaceOfRevolution<PortalSpace2d>*) getSpace()));
	//The following assertions assume getInvert() is true.
	//assert(fabs(getSpace()->getGeodesic(testPointOfReference, otherTestPoint)->getVector().norm() - log(getEDist(t))) < EPSILON);
	assert(fabs(fabs(log(getEDist(tt)/getEDist(t))) - transport->getDistance()) < EPSILON);
	//std::cout << "PortalSpace2d2.cpp getSpace()->getGeodesic(testPointOfReference, out)->getVector().norm():\t" << getSpace()->getGeodesic(testPointOfReference, out)->getVector().norm() << std::endl;
	//std::cout << "transport->getDistance():\t" << transport->getDistance() << std::endl;
	assert(fabs(getSpace()->getGeodesic(testPointOfReference, out)->getVector().norm() - transport->getDistance()) < EPSILON);
	#endif
	assert(containsPoint(out.get()));
	return out;
}


