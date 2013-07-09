#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Euclidean2.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

Euclidean::Point::Point() {
}

//std::tr1::array<double,3> Euclidean::Point::coordinates;
Euclidean::Point::Point(double x, double y, double z, Euclidean* space) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	this->space = space;
}
//Euclidean::Point::~Point() {}

Euclidean* Euclidean::Point::getSpace() {
	return space;
}

//Euclidean::Point Euclidean::PointOfReference::position;
//Matrix3d Euclidean::PointOfReference::orientation;

/*Manifold::Point Euclidean::Point::midpoint(Manifold::Point point) {
	
}*/

std::vector<Vector3d> Euclidean::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {		//They have to have the same parent.
	std::tr1::array<double,3> c0 = position.coordinates;
	std::tr1::array<double,3> c1 = std::tr1::static_pointer_cast<Euclidean::Point>(point)->coordinates;
        Matrix3d test();
	Vector3d base = orientation*Vector3d(c1[0]-c0[0],c1[1]-c0[1],c1[2]-c0[2]);
	std::vector<Vector3d> out(1,base);
	return out;
}

std::tr1::shared_ptr<Manifold::Point> Euclidean::PointOfReference::pointFromVector(Vector3d vector) {
	//std::cout << position.coordinates[0]+vector[0] << "," << position.coordinates[1]+vector[1] << "," << position.coordinates[2]+vector[2] << "\n";
	return std::tr1::shared_ptr<Manifold::Point>(new Euclidean::Point(position.coordinates[0]+vector[0], position.coordinates[1]+vector[1], position.coordinates[2]+vector[2], position.space));
}

Euclidean::PointOfReference::PointOfReference(Euclidean* space) {
	orientation = orientation.Identity();
	position.space = space;
}

Euclidean::PointOfReference::~PointOfReference() {
}

Manifold::Point* Euclidean::PointOfReference::getPosition() {
	return &position;
}

void Euclidean::PointOfReference::move(Vector3d dir) {
	dir = orientation*dir;
	position.coordinates[0] += dir[0];
	position.coordinates[1] += dir[1];
	position.coordinates[2] += dir[2];
	//std::cout << "(" << dir[0] << ", " << dir[1] << ", " << dir[2] << ")\n" << std::flush;
	//std::cout << "(" << position.getCoordinates()[0] << ", " << position.getCoordinates()[1] << ", " << position.getCoordinates()[2] << ")\n" << std::flush;
}

void Euclidean::PointOfReference::rotate(Matrix3d rot) {
	orientation = rot*orientation;
}

