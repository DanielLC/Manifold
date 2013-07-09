#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "R2xS1.h"
#include <Eigen/Dense>
using Eigen::Matrix3d;
using Eigen::Vector3d;

R2xS1::Point::Point() {
}

//std::tr1::array<double,3> R2xS1::Point::coordinates;
R2xS1::Point::Point(double x, double y, double z, R2xS1* space) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	this->space = space;
}
//R2xS1::Point::~Point() {}

R2xS1* R2xS1::Point::getSpace() {
	return space;
}

//R2xS1::Point R2xS1::PointOfReference::position;
//Matrix3d R2xS1::PointOfReference::orientation;

/*Manifold::Point R2xS1::Point::midpoint(Manifold::Point point) {
	
}*/

std::vector<Vector3d> R2xS1::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {		//They have to have the same parent.
	const int SIZE = 32;
	std::tr1::array<double,3> c0 = position.coordinates;
	std::tr1::array<double,3> c1 = std::tr1::static_pointer_cast<R2xS1::Point>(point)->coordinates;
        Matrix3d test();
	Vector3d base = orientation*Vector3d(c1[0]-c0[0],c1[1]-c0[1],c1[2]-c0[2]);
	std::vector<Vector3d> out(SIZE,base);
	for(int i=0; i<SIZE; ++i) {
		//std::cout << i << "\n" << std::flush;
		out[i][0] += i-(SIZE/2);
	}
	return out;
}

std::tr1::shared_ptr<Manifold::Point> R2xS1::PointOfReference::pointFromVector(Vector3d vector) {
	//std::cout << position.coordinates[0]+vector[0] << "," << position.coordinates[1]+vector[1] << "," << position.coordinates[2]+vector[2] << "\n";
	return std::tr1::shared_ptr<Manifold::Point>(new R2xS1::Point(position.coordinates[0]+vector[0], position.coordinates[1]+vector[1], position.coordinates[2]+vector[2], position.space));
}

R2xS1::PointOfReference::PointOfReference(R2xS1* space) {
	orientation = orientation.Identity();
	position.space = space;
}

R2xS1::PointOfReference::~PointOfReference() {
}

Manifold::Point* R2xS1::PointOfReference::getPosition() {
	return &position;
}

void R2xS1::PointOfReference::move(Vector3d dir) {
	dir = orientation*dir;
	position.coordinates[0] += dir[0];
	position.coordinates[0] = fmod(position.coordinates[0],1.);
	position.coordinates[1] += dir[1];
	position.coordinates[2] += dir[2];
	//std::cout << "(" << dir[0] << ", " << dir[1] << ", " << dir[2] << ")\n" << std::flush;
	//std::cout << "(" << position.getCoordinates()[0] << ", " << position.getCoordinates()[1] << ", " << position.getCoordinates()[2] << ")\n" << std::flush;
}

void R2xS1::PointOfReference::rotate(Matrix3d rot) {
	orientation = rot*orientation;
}

