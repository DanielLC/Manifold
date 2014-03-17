#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic2d.h"
#include "Hyperbolic.h"
#include "Assert.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

Hyperbolic::Point::Point(double x, double y, double z, Hyperbolic* space) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	this->space = space;
}

Hyperbolic::Point::Point(Hyperbolic* space) {
	coordinates[2] = 1;
	this->space = space;
}

Hyperbolic::Point::Point() {
	coordinates[2] = 1;
}

Hyperbolic* Hyperbolic::Point::getSpace() {
	return space;
}

std::tr1::array<double,3> Hyperbolic::Point::getCoordinates(){
	return coordinates;
}

Vector3d Hyperbolic::Point::getVector(){
	return Vector3d(coordinates[0],coordinates[1],coordinates[2]);
}

Manifold::GeodesicPtr Hyperbolic::getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end) {
	assert(start->getPosition()->getSpace() == this);
	assert(end->getSpace() == this);
	std::tr1::array<double,3> x = std::tr1::static_pointer_cast<Hyperbolic::Point>(start->getPosition())->getCoordinates();
	Hyperbolic::PointPtr castedEnd = std::tr1::static_pointer_cast<Hyperbolic::Point>(end);
	std::tr1::array<double,3> y = castedEnd->getCoordinates();
	double v0 = y[0]-x[0];
	double v1 = y[1]-x[1];
	double y0 = sqrt(v0*v0+v1*v1);
	v0 /= y0;
	v1 /= y0;
	Hyperbolic2d::Point xpoint(0,x[2]);
	Hyperbolic2d::Point ypoint(y0,y[2]);
	std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic2d = xpoint.getGeodesic(ypoint);
	Vector2d z = geodesic2d->getVector();
	Vector3d vector(v0*z[0],v1*z[0],z[1]);
	vector = std::tr1::static_pointer_cast<Hyperbolic::PointOfReference>(start)->getOrientation().transpose()*vector;
	return Manifold::GeodesicPtr(new Hyperbolic::Geodesic(start, castedEnd, vector, v0, v1, geodesic2d));
}

Matrix3d Hyperbolic::PointOfReference::getOrientation() {
	return orientation;
}

Vector3d Hyperbolic::Geodesic::getVector() {
	return vector;
}

Manifold::GeodesicPtr Hyperbolic::getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector) {
	assert(start->getPosition()->getSpace() == this);
	Vector3d vector2 = std::tr1::static_pointer_cast<Hyperbolic::PointOfReference>(start)->getOrientation()*vector;
	std::tr1::array<double,3> x = std::tr1::static_pointer_cast<Hyperbolic::Point>(start->getPosition())->getCoordinates();
	Hyperbolic2d::Point xpoint(0,x[2]);
	Vector2d z(sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]), vector2[2]);
	double v0 = vector2[0]/z[0];
	double v1 = vector2[1]/z[0];
	//std::cout << "getGeodesic:	" << x[2] << "\n" << z << std::endl;
	std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic2d = xpoint.getGeodesic(z);
	std::tr1::array<double,2> y = geodesic2d->getEndPoint().getCoordinates();
	//std::cout << "Point:	(" << (x[0]+y[0]*v0) << ",	" << (x[1]+y[0]*v1) << ",	" << y[1] << ")\n";
	Hyperbolic::PointPtr end(new Hyperbolic::Point(x[0]+y[0]*v0,x[1]+y[0]*v1,y[1],this));
	//std::cout << "move:" << std::endl;
	//std::cout << "xpoint.getCoordinates()[1]:	" << xpoint.getCoordinates()[1] << std::endl;
	
	//std::cout << "this->getPosition()->getCoordinates()[2]:	" << std::tr1::static_pointer_cast<Hyperbolic::Point>(this->getPosition())->getCoordinates()[2] << std::endl;
	//std::cout << "getGeodesic:" << std::endl;
	//std::cout << "geodesic2d->start:	" << geodesic2d->start << std::endl;
	//std::cout << "geodesic2d->start.getCoordinates()[1]:	" << geodesic2d->start.getCoordinates()[1] << std::endl;

	return Manifold::GeodesicPtr(new Hyperbolic::Geodesic(start, end, vector, v0, v1, geodesic2d));
}

Manifold::PointPtr Hyperbolic::Geodesic::getEndPoint() {
	return end;
}

void Hyperbolic::Point::setCoordinates(double x, double y, double z) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
}

Hyperbolic::PointOfReference::PointOfReference(Hyperbolic::PointPtr position, Matrix3d orientation) {
	this->position = position;
	this->orientation = orientation;
}

Hyperbolic::PointOfReference::PointOfReference(Hyperbolic* space) {
	orientation = orientation.Identity();
	position = Hyperbolic::PointPtr(new Hyperbolic::Point(space));
}

Manifold::PointPtr Hyperbolic::PointOfReference::getPosition() {
	return position;
}

Manifold::PointOfReferencePtr Hyperbolic::getPointOfReference(Manifold::PointOfReferencePtr start, Vector3d vector) {
	return getGeodesic(start, vector)->getEndPointOfReference();
}

Manifold::PointOfReferencePtr Hyperbolic::Geodesic::getEndPointOfReference() {
	double rot = geodesic2d->getRot();	
	Matrix3d rotation = (Matrix3d() << v0,v1,0,-v1,v0,0,0,0,1).finished();
	rotation = (Matrix3d() << cos(rot),0,-sin(rot),0,1,0,sin(rot),0,cos(rot)).finished()*rotation;
	rotation = (Matrix3d() << v0,-v1,0,v1,v0,0,0,0,1).finished()*rotation;
	
	Matrix3d orientation = std::tr1::static_pointer_cast<Hyperbolic::PointOfReference>(start)->getOrientation();
	Hyperbolic::PointOfReferencePtr out(new Hyperbolic::PointOfReference(std::tr1::static_pointer_cast<Hyperbolic::Point>(this->getEndPoint()), rotation*orientation));
	return std::tr1::static_pointer_cast<Manifold::PointOfReference>(out);
}

void Hyperbolic::PointOfReference::rotate(Matrix3d rot) {
	orientation = rot*orientation;
}

Hyperbolic::Geodesic::Geodesic(Manifold::PointOfReferencePtr start, Hyperbolic::PointPtr end, Vector3d vector, double v0, double v1, std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic2d) {
	this->start = start;
	this->end = end;
	this->vector = vector;
	this->v0 = v0;
	this->v1 = v1;
	this->geodesic2d = geodesic2d;
}

