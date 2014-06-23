#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Compound.h"
#include "Manifold.h"
#include "Euclidean.h"
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Assert.h"
using Eigen::Matrix3d;
using Eigen::Vector3d;

Compound::Point::Point() {
}

Compound::Point::Point(std::tr1::shared_ptr<Manifold::Point> position) {
	this->position = position;
}

/*Vector3d Compound::Point::getVector() {
	//This function should never get called. Creating an appropriate vector is impossible, so we must simply avoid making Compounds of Compounds.
	assert(0);
	return Vector3d();
}*/

Manifold* Compound::Point::getSubspace() {
	return position->getSpace();
}

Manifold::PointPtr Compound::Point::getPosition() {
	return position;
}

std::pair<bool,Vector3d> Compound::Point::getVector(Manifold* space, int i) {
	//std::cout << "Compound.cpp space:\t" << getPosition()->getSpace()->getType() << std::endl;
	//std::cout << "Compound.cpp i:\t" << i << std::endl;
	//std::cout << "Compound.cpp getPosition()->getSpace():\t" << getPosition()->getSpace() << std::endl;
	//std::cout << "Compound.cpp space:\t" << space << std::endl;
	if(i <= 0) {
		//std::cout << "Compound.cpp i <= 0" << std::endl;
		if(getPosition()->getSpace() == space) {
			//std::cout << "Compound.cpp getPosition()->getSpace() == space" << std::endl;
			return std::make_pair(true,getPosition()->getVector());
		} else {
			//std::cout << "Compound.cpp getPosition()->getSpace() != space" << std::endl;
			return std::make_pair(false,Vector3d());
		}
	}
	//std::cout << "Compound.cpp i > 0" << std::endl;
	std::vector<Manifold::PortalPtr>* portals = getPosition()->getSpace()->getPortals();
	for(int j=0; j<portals->size(); ++j) {
		std::pair<bool,Vector3d> out = Compound::Point((*portals)[j]->teleport(getPosition())).getVector(space, i-1);
		if(out.first) {
			return out;
		}
	}
	return std::make_pair(false,Vector3d());
}

Vector3d Compound::Point::getVector(Manifold* space) {
	for(int i=0; i<5; ++i) {
		std::pair<bool,Vector3d> out = getVector(space, i);
		if(out.first) {
			//std::cout << "Compound.cpp Vector:\t" << out.second << std::endl;
			return out.second;
		}
	}
	std::cout << "Compound.cpp No connection to space found." << std::endl;
	return Vector3d(0,0,0);
}

Vector3d Compound::PointOfReference::vectorFromPointAndNearVector(Compound::PointPtr point, Vector3d vector) {
	return vectorFromPointAndNearVector(point, vector, 0);
}

Vector3d Compound::PointOfReference::vectorFromPointAndNearVector(Compound::PointPtr point, Vector3d vector, int i) {
	if(i > 100) {
		//std::cout << "Compound.cpp vector:\n" << vector << std::endl;
		//return vector;
		return Vector3d(0,0,0);
	}
	if(vector != vector) {
		return Vector3d(0,0,0);
	}
	Vector3d v1 = point->getPosition()->getVector();
	//Vector3d v0 = point->getPosition()->getVector();
	//assert(v0 == v0);
	double epsilon = 0.00001;
	Manifold* space = point->getPosition()->getSpace();
	//std::cout << "Compound.cpp i:\t" << i << std::endl;
	Vector3d v0 = this->pointFromVector(vector)->getVector(space);
	Vector3d vx = this->pointFromVector(vector + Vector3d(epsilon,0,0))->getVector(space);
	Vector3d vy = this->pointFromVector(vector + Vector3d(0,epsilon,0))->getVector(space);
	Vector3d vz = this->pointFromVector(vector + Vector3d(0,0,epsilon))->getVector(space);
	assert(vz == vz);
	Matrix3d jacobean;
	jacobean << vx-v0,vy-v0,vz-v0;
	jacobean /= epsilon;
	/*std::cout << "getPosition()->getVector():\n" << getPosition()->getVector() << std::endl;
	std::cout << "vector:\n" << vector << std::endl;
	std::cout << "v0:\n" << v0 << std::endl;
	std::cout << "vx:\n" << vx << std::endl;
	std::cout << "vy:\n" << vy << std::endl;
	std::cout << "vz:\n" << vz << std::endl;*/
	//std::cout << "jacobean:\n" << jacobean << std::endl;
	Vector3d delta = jacobean.inverse()*(v1-v0);
	//std::cout << "v1-v0:\n" << v1-v0 << std::endl;
	//std::cout << "delta:\n" << delta << std::endl;
	if(delta != delta) {
		return Vector3d(0,0,0);
	}
	//std::cout << "delta.norm():\t" << delta.norm() << std::endl;
	double squaredNorm = delta.squaredNorm();
	double max = 5;
	if(squaredNorm > max*max) {
		delta = max*delta/sqrt(squaredNorm);
	}
	if(squaredNorm < EPSILON*EPSILON) {
		/*std::cout << "v1-v0:\n" << v1-v0 << std::endl;
		std::cout << "delta:\n" << delta << std::endl;
		std::cout << "vector:\n" << vector << std::endl;
		std::cout << "delta+vector:\n" << delta+vector << std::endl;
		std::cout << "pointOfReference->vectorFromPoint(point):\n" << pointOfReference->getGeodesic(point->getPosition())->getVector() << std::endl;*/
		assert(pointFromVector(delta+vector)->getPosition()->getSpace() == point->getPosition()->getSpace());
		assert((pointFromVector(delta+vector)->getPosition()->getVector() - point->getPosition()->getVector()).squaredNorm() < EPSILON);
		/*assert((pointOfReference->getSpace()->pointFromVector(pointOfReference, vector)->getVector() - point->getPosition()->getVector()).squaredNorm() < EPSILON);
		assert((this->pointFromVector(delta+vector)->getPosition()->getVector() - pointOfReference->getSpace()->pointFromVector(pointOfReference, vector)->getVector()).squaredNorm() < EPSILON);
		assert((delta+vector - pointOfReference->getSpace()->vectorFromPoint(pointOfReference, point->getPosition())).squaredNorm() < EPSILON);*/	//Only for portals that connect to themselves.
		//std::cout << "i:\t" << i << std::endl;
		return delta+vector;
	} else {
		return vectorFromPointAndNearVector(point, delta+vector, i+1);
	}
}

Manifold::GeodesicPtr Compound::PointOfReference::getFinalGeodesic(Vector3d vector) {
	/*Manifold* space = pointOfReference->getPosition()->getSpace();
	return space->getGeodesic(pointOfReference, vector);*/
	//std::cout << "getFinalGeodesic(vector)" << std::endl;
	assert(vector == vector);
	//This will need to be made more sophisticated when Compound is made to support more than one space.
	Manifold* space = pointOfReference->getPosition()->getSpace();
	Manifold::GeodesicPtr next = space->getGeodesic(pointOfReference, vector);
	//Manifold::GeodesicPtr original = next;
	//Vector3d firstVector = next->getEndPoint()->getVector();
	Manifold::GeodesicPtr current;
	do {
		current = next;
		space = current->getSpace();
		next = space->nextPiece(current);
		/*if(next) {
			std::cout << "next" << std::endl;
		}*/
	} while(next);
	//assert((current->getEndPoint()->getVector() - firstVector).squaredNorm() < EPSILON*EPSILON);	//For portals leading to themselves.
	//std::cout << "current->getEndPoint()->getVector():\n" << current->getEndPoint()->getVector() << std::endl;
	//std::cout << "firstVector:\n" << firstVector << std::endl;
	//return original;
	assert(current->getEndPoint()->isInManifold());
	return current;
}

Compound::PointPtr Compound::PointOfReference::pointFromVector(Vector3d vector) {
	Manifold::GeodesicPtr geodesic = getFinalGeodesic(vector);
	Compound::PointPtr out(new Compound::Point(geodesic->getEndPoint()));
	return out;
}

Compound::PointOfReferencePtr Compound::PointOfReference::pointOfReferenceFromVector(Vector3d vector) {
	Manifold::GeodesicPtr geodesic = getFinalGeodesic(vector);
	Compound::PointOfReferencePtr out(new Compound::PointOfReference(geodesic->getEndPointOfReference()));
	return out;
}

Compound::PointOfReference::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference) {
	this->pointOfReference = pointOfReference;
}

/*Compound::PointOfReference::~PointOfReference() {
}*/

Manifold::PointPtr Compound::PointOfReference::getPosition() {
	return pointOfReference->getPosition();
}

void Compound::PointOfReference::move(Vector3d dir) {
	//pointOfReference = pointOfReference->getSpace()->getPointOfReference(pointOfReference, dir);
	//std::cout << std::tr1::static_pointer_cast<Euclidean::PointOfReference>(pointOfReference)->getCoordinates() << std::endl;
	//assert((pointOfReference->getSpace()->getPointOfReference(pointOfReference, dir)->getPosition()->getVector() - getFinalGeodesic(dir)->getEndPoint()->getVector()).squaredNorm() < EPSILON);	//Only for if the portal leads to itself.
	//std::cout << "Compound.cpp moving" << std::endl;
	pointOfReference = getFinalGeodesic(dir)->getEndPointOfReference();
	//std::cout << "Compound.cpp moved" << std::endl;
	//std::cout << "Compound.cpp vector:\n" << pointOfReference->getPosition()->getVector() << std::endl;
	//std::cout << "Compound.cpp camera position:\n" << pointOfReference->getPosition()->getVector() << std::endl;
}

void Compound::PointOfReference::rotate(Matrix3d rot) {
	pointOfReference->rotate(rot);
}

std::vector<Compound::Triangle> Compound::PointOfReference::icosahedron() {
	return icosahedron(1.);
}

std::vector<Compound::Triangle> Compound::PointOfReference::octahedron() {
	return octahedron(1.);
}

Compound::PointPtr Compound::PointOfReference::cylindrical(double r, double theta, double h) {
	Compound::PointOfReferencePtr height = pointOfReferenceFromVector(Vector3d(0,0,h));
	return height->pointFromVector(Vector3d(r*cos(theta),r*sin(theta),0));
}

std::vector<Compound::Triangle> Compound::PointOfReference::helix(double k) {
	double r = k/4;
	double dtheta = M_PI*0.9;
	double dh = k/10;
	double theta = 0;
	double h = -k;
	std::vector<Compound::Triangle> out;
	Compound::PointPtr v0 = cylindrical(r, theta, h);
	theta += dtheta;
	h += dh;
	Compound::PointPtr v1 = cylindrical(r, theta, h);
	for(int i=-8; i<=10; ++i) {
		theta += dtheta;
		h += dh;
		Compound::PointPtr v2 = cylindrical(r, theta, h);
		Triangle triangle = {v0,v1,v2};
		out.push_back(triangle);
		v0 = v1;
		v1 = v2;
	}
	return out;
}

std::vector<Compound::Triangle> Compound::PointOfReference::octahedron(double k) {
	std::vector<Compound::Triangle> out;
	Compound::PointPtr xp = pointFromVector(Vector3d(1,0,0));
	Compound::PointPtr xm = pointFromVector(Vector3d(-1,0,0));
	Compound::PointPtr yp = pointFromVector(Vector3d(0,1,0));
	Compound::PointPtr ym = pointFromVector(Vector3d(0,-1,0));
	Compound::PointPtr zp = pointFromVector(Vector3d(0,0,1));
	Compound::PointPtr zm = pointFromVector(Vector3d(0,0,-1));
	Triangle triangle0 = {xp,yp,zp};
	Triangle triangle1 = {xp,yp,zm};
	Triangle triangle2 = {xp,ym,zp};
	Triangle triangle3 = {xp,ym,zm};
	Triangle triangle4 = {xm,yp,zp};
	Triangle triangle5 = {xm,yp,zm};
	Triangle triangle6 = {xm,ym,zp};
	Triangle triangle7 = {xm,ym,zm};
	out.push_back(triangle0);
	out.push_back(triangle1);
	out.push_back(triangle2);
	out.push_back(triangle3);
	out.push_back(triangle4);
	out.push_back(triangle5);
	out.push_back(triangle6);
	out.push_back(triangle7);
	return out;
}

std::vector<Compound::Triangle> Compound::PointOfReference::icosahedron(double k) {
	double phi = k*(-1.+sqrt(5.))/2.;
	std::vector<Compound::Triangle> out;
	Compound::PointPtr va0 = pointFromVector(Vector3d(0,k,phi));
	Compound::PointPtr va1 = pointFromVector(Vector3d(0,k,-phi));
	Compound::PointPtr va2 = pointFromVector(Vector3d(0,-k,phi));
	Compound::PointPtr va3 = pointFromVector(Vector3d(0,-k,-phi));
	Compound::PointPtr vb0 = pointFromVector(Vector3d(k,phi,0));
	Compound::PointPtr vb1 = pointFromVector(Vector3d(k,-phi,0));
	Compound::PointPtr vb2 = pointFromVector(Vector3d(-k,phi,0));
	Compound::PointPtr vb3 = pointFromVector(Vector3d(-k,-phi,0));
	Compound::PointPtr vc0 = pointFromVector(Vector3d(phi,0,k));
	Compound::PointPtr vc1 = pointFromVector(Vector3d(-phi,0,k));
	Compound::PointPtr vc2 = pointFromVector(Vector3d(phi,0,-k));
	Compound::PointPtr vc3 = pointFromVector(Vector3d(-phi,0,-k));
	Triangle triangle0 = {va0,va1,vb0};
	Triangle triangle1 = {va0,va1,vb2};
	Triangle triangle2 = {va2,va3,vb1};
	Triangle triangle3 = {va2,va3,vb3};
	Triangle triangle4 = {vb0,vb1,vc0};
	Triangle triangle5 = {vb0,vb1,vc2};
	Triangle triangle6 = {vb2,vb3,vc1};
	Triangle triangle7 = {vb2,vb3,vc3};
	Triangle triangle8 = {vc0,vc1,va0};
	Triangle triangle9 = {vc0,vc1,va2};
	Triangle triangle10 = {vc2,vc3,va1};
	Triangle triangle11 = {vc2,vc3,va3};
	out.push_back(triangle0);
	out.push_back(triangle1);
	out.push_back(triangle2);
	out.push_back(triangle3);
	out.push_back(triangle4);
	out.push_back(triangle5);
	out.push_back(triangle6);
	out.push_back(triangle7);
	out.push_back(triangle8);
	out.push_back(triangle9);
	out.push_back(triangle10);
	out.push_back(triangle11);
	return out;
}

