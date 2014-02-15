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

std::tr1::shared_ptr<Manifold::Point> Compound::Point::getPosition() {
	return position;
}

Vector3d Compound::PointOfReference::vectorFromPointAndNearVector(std::tr1::shared_ptr<Compound::Point> point, Vector3d vector) {
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

std::tr1::shared_ptr<Compound::Point> Compound::PointOfReference::pointFromVector(Vector3d vector) {
	assert(vector == vector);
	//This will need to be made more sophisticated when Compound is made to support more than one space.
	std::tr1::shared_ptr<Compound::Point> out(new Compound::Point(pointOfReference->pointFromVector(vector)));
	return out;
}

Compound::PointOfReference::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference) {
	this->pointOfReference = pointOfReference;
}

Compound::PointOfReference::~PointOfReference() {
}

Manifold::Point* Compound::PointOfReference::getPosition() {
	return pointOfReference->getPosition();
}

void Compound::PointOfReference::move(Vector3d dir) {
	pointOfReference->move(dir);
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

std::vector<Compound::Triangle> Compound::PointOfReference::octahedron(double k) {
	std::vector<Compound::Triangle> out;
	std::tr1::shared_ptr<Compound::Point> xp = pointFromVector(Vector3d(1,0,0));
	std::tr1::shared_ptr<Compound::Point> xm = pointFromVector(Vector3d(-1,0,0));
	std::tr1::shared_ptr<Compound::Point> yp = pointFromVector(Vector3d(0,1,0));
	std::tr1::shared_ptr<Compound::Point> ym = pointFromVector(Vector3d(0,-1,0));
	std::tr1::shared_ptr<Compound::Point> zp = pointFromVector(Vector3d(0,0,1));
	std::tr1::shared_ptr<Compound::Point> zm = pointFromVector(Vector3d(0,0,-1));
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
	std::tr1::shared_ptr<Compound::Point> va0 = pointFromVector(Vector3d(0,k,phi));
	std::tr1::shared_ptr<Compound::Point> va1 = pointFromVector(Vector3d(0,k,-phi));
	std::tr1::shared_ptr<Compound::Point> va2 = pointFromVector(Vector3d(0,-k,phi));
	std::tr1::shared_ptr<Compound::Point> va3 = pointFromVector(Vector3d(0,-k,-phi));
	std::tr1::shared_ptr<Compound::Point> vb0 = pointFromVector(Vector3d(k,phi,0));
	std::tr1::shared_ptr<Compound::Point> vb1 = pointFromVector(Vector3d(k,-phi,0));
	std::tr1::shared_ptr<Compound::Point> vb2 = pointFromVector(Vector3d(-k,phi,0));
	std::tr1::shared_ptr<Compound::Point> vb3 = pointFromVector(Vector3d(-k,-phi,0));
	std::tr1::shared_ptr<Compound::Point> vc0 = pointFromVector(Vector3d(phi,0,k));
	std::tr1::shared_ptr<Compound::Point> vc1 = pointFromVector(Vector3d(-phi,0,k));
	std::tr1::shared_ptr<Compound::Point> vc2 = pointFromVector(Vector3d(phi,0,-k));
	std::tr1::shared_ptr<Compound::Point> vc3 = pointFromVector(Vector3d(-phi,0,-k));
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
