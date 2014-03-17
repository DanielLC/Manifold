#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Euclidean.h"
#include <Eigen/Core>
#include <utility>

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
	return Manifold::GeodesicPtr(new Euclidean::Geodesic(castedStart, castedEnd, castedEnd->getCoordinates()-castedStart->getCoordinates()));
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
	Euclidean::PointPtr end(new Euclidean::Point(castedStart->getCoordinates() + vector, (Euclidean*) start->getSpace()));
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
	orientation = rot*orientation;
}

Euclidean::Geodesic::Geodesic(Euclidean::PointOfReferencePtr start, Euclidean::PointPtr end, Vector3d vector) {
	this->start = start;
	this->end = end;
	this->vector = vector;
}

