#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic2d.h"
#include "Hyperbolic.h"
#include "Assert.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

Hyperbolic::Point::Point() {
	coordinates[2] = 1;
}

//std::tr1::array<double,3> Hyperbolic::Point::coordinates;
Hyperbolic::Point::Point(double x, double y, double z, Hyperbolic* space) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	this->space = space;
}
//Hyperbolic::Point::~Point() {}

Manifold* Hyperbolic::Point::getSpace() {
	return space;
}

std::tr1::array<double,3> Hyperbolic::Point::getCoordinates(){return coordinates;}

Vector3d Hyperbolic::Point::getVector(){return Vector3d(coordinates[0],coordinates[1],coordinates[2]);}

//Hyperbolic::Point Hyperbolic::PointOfReference::position;
//Matrix3d Hyperbolic::PointOfReference::orientation;

/*Manifold::Point Hyperbolic::Point::midpoint(Manifold::Point point) {
	
}*/

Vector3d Hyperbolic::PointOfReference::vectorFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {
	std::tr1::array<double,3> x = position.coordinates;
	std::tr1::array<double,3> y = std::tr1::static_pointer_cast<Hyperbolic::Point>(point)->coordinates;
	double v0 = y[0]-x[0];
	double v1 = y[1]-x[1];
	double y0 = sqrt(v0*v0+v1*v1);
	v0 /= y0;
	v1 /= y0;
	Hyperbolic2d::Point xpoint(0,x[2]);
	Hyperbolic2d::Point ypoint(y0,y[2]);
	Vector2d z = xpoint.vectorFromPoint(ypoint);
	Vector3d out(v0*z[0],v1*z[0],z[1]);
	out = orientation.transpose()*out;
	return out;
}

std::vector<Vector3d> Hyperbolic::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {
	std::vector<Vector3d> out;
	out.push_back(this->vectorFromPoint(point));
	return out;
}

std::tr1::shared_ptr<Manifold::Point> Hyperbolic::PointOfReference::pointFromVector(Vector3d vector) {
	assert(vector == vector);
	vector = orientation*vector;
	std::tr1::array<double,3> x = position.coordinates;
	Hyperbolic2d::Point xpoint(0,x[2]);
	Vector2d z(sqrt(vector[0]*vector[0]+vector[1]*vector[1]), vector[2]);
	assert(z == z);
	double v0;
	double v1;
	if(fabs(z[0]) > EPSILON) {
		v0 = vector[0]/z[0];
		v1 = vector[1]/z[0];
	} else {	//It doesn't really matter, but I'd recommend making sure the magnitude is one.
		v0 = 1;
		v1 = 0;
	}
	std::tr1::array<double,2> y = xpoint.pointFromVector(z).getCoordinates();
	//std::cout << "Point:	(" << (x[0]+y[0]*v0) << ",	" << (x[1]+y[0]*v1) << ",	" << y[1] << ")\n";
	return std::tr1::shared_ptr<Manifold::Point>(new Hyperbolic::Point(x[0]+y[0]*v0,x[1]+y[0]*v1,y[1],position.space));
}

void Hyperbolic::Point::setCoordinates(double x, double y, double z) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
}

Hyperbolic::PointOfReference::PointOfReference(Hyperbolic* space) {
	orientation = orientation.Identity();
	position.space = space;
}

Hyperbolic::PointOfReference::~PointOfReference() {
}

Manifold::Point* Hyperbolic::PointOfReference::getPosition() {
	return &position;
}

void Hyperbolic::PointOfReference::move(Vector3d vector) {
	vector = orientation*vector;
	std::tr1::array<double,3> x = position.coordinates;
	Hyperbolic2d::Point xpoint(0,x[2]);
	Vector2d z(sqrt(vector[0]*vector[0]+vector[1]*vector[1]), vector[2]);
	double v0 = vector[0]/z[0];
	double v1 = vector[1]/z[0];
	std::pair<Hyperbolic2d::Point, double> pointAndRot = xpoint.pointAndRotFromVector(z);
	std::tr1::array<double,2> y = pointAndRot.first.getCoordinates();
	double rot = pointAndRot.second;
	position.setCoordinates(x[0]+y[0]*v0,x[1]+y[0]*v1,y[1]);
	
	orientation = (Matrix3d() << v0,v1,0,-v1,v0,0,0,0,1).finished()*orientation;
	orientation = (Matrix3d() << cos(rot),0,-sin(rot),0,1,0,sin(rot),0,cos(rot)).finished()*orientation;
	orientation = (Matrix3d() << v0,-v1,0,v1,v0,0,0,0,1).finished()*orientation;
	//std::cout << "determinant0:	" << (Matrix3d() << cos(rot),0,-sin(rot),0,1,0,sin(rot),0,cos(rot)).finished().determinant() << "\n";
	//std::cout << "determinant1:	" << (Matrix3d() << v0,-v1,0,v1,v0,0,0,0,1).finished().determinant() << "\n";
	//std::cout << "determinant:	" << orientation.determinant() << "\n";
}

void Hyperbolic::PointOfReference::rotate(Matrix3d rot) {
	orientation = rot*orientation;
}

