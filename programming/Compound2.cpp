//Depricated

#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Compound.h"
#include "Manifold.h"
#include <Eigen/Core>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Assert.h"
using Eigen::Matrix3d;
using Eigen::Vector3d;

Manifold* Compound::getSpace() {
	return space;
}

Compound::Point::Point(std::tr1::shared_ptr<Manifold::Point> position, Compound* space) {
	this->position = position;
	this->space = space;
}

Vector3d Compound::Point::getVector() {
	//This function should never get called. Creating an appropriate vector is impossible, so we must simply avoid making Compounds of Compounds.
	assert(0);
	return Vector3d();
}

Manifold* Compound::Point::getSubspace() {
	return position->getSpace();
}

std::tr1::shared_ptr<Manifold::Point> Compound::Point::getPosition() {
	return position;
}

Vector3d Compound::PointOfReference::vectorFromPointAndNearVector(Compound::PointPtr point, Vector3d vector) {
	assert(vector == vector);
	Vector3d v1 = point->getPosition()->getVector();
	//Vector3d v0 = point->getPosition()->getVector();
	//assert(v0 == v0);
	double epsilon = 0.001;
	Vector3d v0 = pointFromVector(vector)->getPosition()->getVector();
	Vector3d vx = pointFromVector(vector + Vector3d(epsilon,0,0))->getPosition()->getVector();
	Vector3d vy = pointFromVector(vector + Vector3d(0,epsilon,0))->getPosition()->getVector();
	Vector3d vz = pointFromVector(vector + Vector3d(0,0,epsilon))->getPosition()->getVector();
	assert(vz == vz);
	Matrix3d jacobean;
	jacobean << vx-v0,vy-v0,vz-v0;
	jacobean /= epsilon;
	jacobean.transposeInPlace();
	/*std::cout << "v0:\n" << v0 << std::endl;
	std::cout << "vx:\n" << vx << std::endl;
	std::cout << "vy:\n" << vy << std::endl;
	std::cout << "vz:\n" << vz << std::endl;
	std::cout << "jacobean:\n" << jacobean << std::endl;*/
	//Matrix3d jacobean = point.getJacobean();
	//Better yet, maybe I'll make it so I can just get the inverse.
	Vector3d delta = jacobean.inverse()*(v1-v0);
	assert(delta == delta);
	if(delta.norm() < EPSILON) {
		//std::cout << "v1-v0:\n" << v1-v0 << std::endl;
		//std::cout << "delta:\n" << delta << std::endl;
		//std::cout << "vector:\n" << vector << std::endl;
		//std::cout << "delta+vector:\n" << delta+vector << std::endl;
		//std::cout << "pointOfReference->vectorFromPoint(point):\n" << pointOfReference->vectorsFromPoint(point->getPosition())[0] << std::endl;
		return delta+vector;
	} else {
		return vectorFromPointAndNearVector(point, delta+vector);
	}
}

/*std::tr1::shared_ptr<Compound::Point> Compound::PointOfReference::pointFromVector(Vector3d vector) {
	assert(vector == vector);
	//This will need to be made more sophisticated when Compound is made to support more than one space.
	std::tr1::shared_ptr<Compound::Point> out(new Compound::Point(pointOfReference->pointFromVector(vector)));
	return out;
}*/

Compound::PointOfReference::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference) {
	this->pointOfReference = pointOfReference;
}

Manifold::PointPtr Compound::PointOfReference::getPosition() {
	return pointOfReference->getPosition();
}

void Compound::PointOfReference::rotate(Matrix3d rot) {
	pointOfReference->rotate(rot);
}

Compound::Geodesic::Geodesic(Compound::PointOfReferencePtr start, Compound::PointOfReferencePtr end, Vector3d vector) {
	this->start = start;
	this->end = end;
	this->vector = vector;
}

Manifold::PointPtr Compound::Geodesic::getEndPoint() {
	return end->getPosition();
}

Manifold::PointOfReferencePtr Compound::Geodesic::getEndPointOfReference() {
	return std::tr1::static_pointer_cast<Manifold::PointOfReference>(end);
}

Vector3d Compound::Geodesic::getVector() {
	return vector;
}

Manifold::geodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end) {
	Compound::PointOfReferencePtr castedStart = std::tr1::static_pointer_cast<Compound::PointOfReference>(start);
	Compound::PointPtr castedEnd = std::tr1::static_pointer_cast<Compound::Point>(end);
	Vector3d vector = start->getSpace()->getGeodesic(start, end)->getVector();
}

