#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Euclidean.h"
#include "Assert.h"
#include <Eigen/Core>
#include <utility>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;

Euclidean::Point::Point(double x, double y, double z, Euclidean* space) {
	coordinates << x,y,z;
	this->space = space;
}

Euclidean::Point::Point(Vector3d coordinates, Euclidean* space) {
	this->coordinates = coordinates;
	this->space = space;
}

Euclidean::Point::Point(Euclidean* space) {
	coordinates << 0,0,0;
	this->space = space;
}

Euclidean::Point::Point() {
	coordinates << 0,0,0;
}

Euclidean* Euclidean::Point::getSpace() {
	return space;
}

Vector3d Euclidean::Point::getCoordinates(){
	return coordinates;
}

Vector3d Euclidean::Point::getVector(){
	return coordinates;
}

Manifold::GeodesicPtr Euclidean::getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end) {
	Euclidean::PointOfReferencePtr castedStart = std::tr1::static_pointer_cast<Euclidean::PointOfReference>(start);
	Euclidean::PointPtr castedEnd = std::tr1::static_pointer_cast<Euclidean::Point>(end);
	return Manifold::GeodesicPtr(new Euclidean::Geodesic(castedStart, castedEnd, castedStart->getOrientation().transpose()*(castedEnd->getCoordinates()-castedStart->getCoordinates())));
}

Matrix3d Euclidean::PointOfReference::getOrientation() {
	return orientation;
}

Vector3d Euclidean::PointOfReference::getCoordinates() {
	return position->getCoordinates();
}

Vector3d Euclidean::Geodesic::getVector() {
	return vector;
}

Manifold::GeodesicPtr Euclidean::getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector) {
	Euclidean::PointOfReferencePtr castedStart = std::tr1::static_pointer_cast<Euclidean::PointOfReference>(start);
	Euclidean::PointPtr end(new Euclidean::Point(castedStart->getCoordinates() + castedStart->getOrientation()*vector, (Euclidean*) start->getSpace()));
	return Manifold::GeodesicPtr(new Euclidean::Geodesic(castedStart, end, vector));
}

Manifold::PointPtr Euclidean::Geodesic::getEndPoint() {
	return std::tr1::static_pointer_cast<Manifold::Point>(end);
}

void Euclidean::Point::setCoordinates(double x, double y, double z) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
}

Euclidean::PointOfReference::PointOfReference(Euclidean::PointPtr position, Matrix3d orientation) {
	this->position = position;
	this->orientation = orientation;
}

Euclidean::PointOfReference::PointOfReference(Euclidean* space) {
	orientation = orientation.Identity();
	position = Euclidean::PointPtr(new Euclidean::Point(space));
}

Manifold::PointPtr Euclidean::PointOfReference::getPosition() {
	return std::tr1::static_pointer_cast<Manifold::Point>(position);
}

Manifold::PointOfReferencePtr Euclidean::Geodesic::getEndPointOfReference() {
	Matrix3d orientation(std::tr1::static_pointer_cast<Euclidean::PointOfReference>(start)->getOrientation());
	return Manifold::PointOfReferencePtr(new Euclidean::PointOfReference(std::tr1::static_pointer_cast<Euclidean::Point>(end), orientation));
}

void Euclidean::PointOfReference::rotate(Matrix3d rot) {
	orientation = orientation*rot;
}

Euclidean::Geodesic::Geodesic(Euclidean::PointOfReferencePtr start, Euclidean::PointPtr end, Vector3d vector) {
	this->start = start;
	this->end = end;
	this->vector = vector;
}

Manifold::PointOfReferencePtr Euclidean::Geodesic::getStart() {
	return start;
}

double Euclidean::Geodesic::intersectionDistance(Manifold::PortalPtr portal) {
	return intersectionDistance2(portal.get());
}

double Euclidean::Geodesic::intersectionDistance2(Manifold::Portal* portal) {	//I just changed this to Manifold, since you apparently can't read and keep insisting that's what the declaration says.
	/*if(portal->getSpace() != this->getSpace()) {
		std::cout << "wrong space" << std::endl;
		return INFINITY;
	}*/
	if(vector.squaredNorm() < EPSILON) {
		//std::cout << "Infinity case 1" << std::endl;
		return INFINITY;
	}

	Euclidean::Portal* castedPortal = (Euclidean::Portal*) portal;
	Vector3d displacement = castedPortal->getCenter() - start->getCoordinates();
	double r = castedPortal->getRadius();
	double dot = (start->getOrientation()*vector).normalized().dot(displacement);			//The distance to the closest the geodesic gets to the center of the portal
	double closestSquared = displacement.squaredNorm() - dot*dot;//The square of the closest the geodesic gets to the center of the portal
	//std::cout << "r*r-closestSquared:\t" << r*r-closestSquared << std::endl;
	if(closestSquared > r*r-EPSILON) {
		/*std::cout << "r:\t" << r << std::endl;
		std::cout << "center:\n" << castedPortal->getCenter() << std::endl;
		std::cout << "Infinity case 2" << std::endl;*/
		return INFINITY;
	}
	double squareroot = sqrt(r*r-closestSquared);
	//std::cout << "dot-squareroot:\t\t" << dot-squareroot << "\ndot+squareroot:\t\t" << dot+squareroot << std::endl;
	if(dot-squareroot > 0.00001) {
		//std::cout << "Euclidean.cpp dot-squareroot:\t" << dot-squareroot << std::endl;
		//Should be this if it's entering the sphere.
		return dot-squareroot;
	} else if(dot+squareroot > 0.00001) {
		//std::cout << "Euclidean.cpp dot+squareroot:\t" << dot+squareroot << std::endl;
		//Should be this if it's leaving the sphere.
		return dot+squareroot;
	} else {
		//std::cout << "dot+squareroot:\t" << dot+squareroot << std::endl;
		//std::cout << "EPSILON:\t" << EPSILON << std::endl;
		//std::cout << "infinity" << std::endl;
		//std::cout << "Infinity case 3" << std::endl;
		return INFINITY;
	}
}

bool Euclidean::Portal::containsPoint(Manifold::Point* point) {
	double x = (((Euclidean::Point*) point)->getCoordinates() - center).squaredNorm() - radius*radius;
	if(getInvert()) {
		return x > EPSILON;
	} else {
		return x < -EPSILON;
	}
}

Euclidean::Portal::Portal(Vector3d center, double radius, Manifold* space) {
	this->center = center;
	this->radius = radius;
	this->setSpace(space);
}

Euclidean::Portal::~Portal() {
	std::cout << "Portal destroyed" << std::endl;
}

Vector3d Euclidean::Portal::getCenter() {
	return center;
}

double Euclidean::Portal::getRadius() {
	return radius;
}

IntersectionPtr Euclidean::Portal::getIntersection(Manifold::GeodesicPtr geodesic) {
	double norm = geodesic->getVector().norm();
	Vector3d normalized = geodesic->getVector()/norm;
	Euclidean::GeodesicPtr castedGeodesic = std::tr1::static_pointer_cast<Euclidean::Geodesic>(geodesic);
	//std::cout << "ac" << std::endl;
	double dist = castedGeodesic->intersectionDistance2(this);
	assert(dist > EPSILON);
	//std::cout << "ad" << std::endl;
	Euclidean::PointOfReference* start = (Euclidean::PointOfReference*) castedGeodesic->getStart().get();
	Vector3d position = (start->getCoordinates() + dist*normalized - center).normalized();
	Matrix3d orientation = start->getOrientation();
	Vector3d vector = (norm-dist)*normalized;
	IntersectionPtr intersection(new Intersection(position, orientation, vector));
	assert(!intersection->getSign() ^ getInvert());
	return intersection;
}

Manifold::GeodesicPtr Euclidean::Portal::getGeodesic(IntersectionPtr intersection) {
	//std::cout << "getGeodesic(IntersectionPtr intersection)" << std::endl;
	PointPtr startPoint(new Euclidean::Point(center + intersection->getPosition()*radius, (Euclidean*)getSpace()));
	PointOfReferencePtr start(new Euclidean::PointOfReference(startPoint, intersection->getOrientation()));
	return getSpace()->getGeodesic(start, intersection->getVector());
}

PointTransportPtr Euclidean::Portal::getTransport(Manifold::PointPtr point) {
	Vector3d delta = ((Euclidean::Point*) point.get())->getCoordinates() - center;
	//std::cout << "Euclidean.cpp delta:\n" << delta << std::endl;
	Vector3d position = delta.normalized();
	double distance = delta.norm() - radius;
	if(getInvert()) {
		distance = -distance;
	}
	return PointTransportPtr(new PointTransport(position, distance));
}

Manifold::PointPtr Euclidean::Portal::getPoint(PointTransportPtr transport) {
	Vector3d coordinates;
	if(getInvert()) {
		coordinates = center + (radius+transport->getDistance())*transport->getPosition();
	} else {
		coordinates = center + radius*exp(-transport->getDistance())*transport->getPosition();
	}
	return Manifold::PointPtr(new Euclidean::Point(coordinates, (Euclidean*) getSpace()));
}

double Euclidean::Portal::getRadiusOfCurvature() {
	return radius;
}

double Euclidean::Portal::getCircumference() {
	return 2*M_PI*radius;
}

std::string Euclidean::getType() {
	return "Euclidean";
}

