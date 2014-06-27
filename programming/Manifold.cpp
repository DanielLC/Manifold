#include "Manifold.h"
#include "Euclidean.h"
#include "SurfaceOfRevolution.h"
#include "PortalSpace2d2.h"
#include "Assert.h"

std::vector<Manifold::Triangle> Manifold::icosahedron(PointOfReferencePtr por) {
	return icosahedron(por, 1.);
}

std::vector<Manifold::Triangle> Manifold::octahedron(PointOfReferencePtr por) {
	return octahedron(por, 1.);
}

std::vector<Manifold::Triangle> Manifold::octahedron(PointOfReferencePtr por, double k) {
	std::vector<Manifold::Triangle> out;
	std::tr1::shared_ptr<Manifold::Point> xp = pointFromVector(por,Vector3d( 1, 0, 0));
	std::tr1::shared_ptr<Manifold::Point> xm = pointFromVector(por,Vector3d(-1, 0, 0));
	std::tr1::shared_ptr<Manifold::Point> yp = pointFromVector(por,Vector3d( 0, 1, 0));
	std::tr1::shared_ptr<Manifold::Point> ym = pointFromVector(por,Vector3d( 0,-1, 0));
	std::tr1::shared_ptr<Manifold::Point> zp = pointFromVector(por,Vector3d( 0, 0, 1));
	std::tr1::shared_ptr<Manifold::Point> zm = pointFromVector(por,Vector3d( 0, 0,-1));
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

std::vector<Manifold::Triangle> Manifold::icosahedron(PointOfReferencePtr por, double k) {
	double phi = k*(-1.+sqrt(5.))/2.;
	std::vector<Manifold::Triangle> out;
	std::tr1::shared_ptr<Manifold::Point> va0 = pointFromVector(por,Vector3d(0,k,phi));
	std::tr1::shared_ptr<Manifold::Point> va1 = pointFromVector(por,Vector3d(0,k,-phi));
	std::tr1::shared_ptr<Manifold::Point> va2 = pointFromVector(por,Vector3d(0,-k,phi));
	std::tr1::shared_ptr<Manifold::Point> va3 = pointFromVector(por,Vector3d(0,-k,-phi));
	std::tr1::shared_ptr<Manifold::Point> vb0 = pointFromVector(por,Vector3d(k,phi,0));
	std::tr1::shared_ptr<Manifold::Point> vb1 = pointFromVector(por,Vector3d(k,-phi,0));
	std::tr1::shared_ptr<Manifold::Point> vb2 = pointFromVector(por,Vector3d(-k,phi,0));
	std::tr1::shared_ptr<Manifold::Point> vb3 = pointFromVector(por,Vector3d(-k,-phi,0));
	std::tr1::shared_ptr<Manifold::Point> vc0 = pointFromVector(por,Vector3d(phi,0,k));
	std::tr1::shared_ptr<Manifold::Point> vc1 = pointFromVector(por,Vector3d(-phi,0,k));
	std::tr1::shared_ptr<Manifold::Point> vc2 = pointFromVector(por,Vector3d(phi,0,-k));
	std::tr1::shared_ptr<Manifold::Point> vc3 = pointFromVector(por,Vector3d(-phi,0,-k));
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

Eigen::Vector3d Manifold::vectorFromPoint(Manifold::PointOfReferencePtr start, Manifold::PointPtr end) {
	return start->getSpace()->getGeodesic(start, end)->getVector();
}

Manifold::PointPtr Manifold::pointFromVector(Manifold::PointOfReferencePtr start, Vector3d vector) {
	return this->getGeodesic(start, vector)->getEndPoint();
}

Manifold::PointOfReferencePtr Manifold::getPointOfReference(Manifold::PointOfReferencePtr start, Vector3d vector) {
	return this->getGeodesic(start, vector)->getEndPointOfReference();
}

bool Manifold::Point::isInManifold() {
	std::vector<Manifold::PortalPtr>* portals = getSpace()->getPortals();
	for(int i=0; i<portals->size(); ++i) {
		if((*portals)[i]->containsPoint(this)) {
			return false;
		}
	}
	return true;
}

Manifold* Manifold::PointOfReference::getSpace() {
	return getPosition()->getSpace();
}

Manifold* Manifold::Geodesic::getSpace() {
	return getEndPoint()->getSpace();
}

Manifold* Manifold::Portal::getSpace() {
	return space;
}

Manifold::Portal::Portal() {
	rotate = false;
	invert = false;
}

void Manifold::Portal::setExit(Manifold::Portal* exit) {
	this->exit = exit;
	rotate = false;
}

void Manifold::Portal::setExit(Manifold::Portal* exit, Matrix3d rotation) {
	this->exit = exit;
	if(rotation == Matrix3d() || rotation == Matrix3d::Identity()) {
		rotate = false;
	} else {
		this->rotation = rotation;
		rotate = true;
	}
}

void Manifold::Portal::setMutualExits(Manifold::Portal* exit) {
	setExit(exit);
	exit->setExit(this);
}

void Manifold::Portal::setMutualExits(Manifold::Portal* exit, Matrix3d rotation) {
	setExit(exit, rotation);
	exit->setExit(this, rotation.transpose());
}

void Manifold::Portal::setSpace(Manifold* space) {
	this->space = space;
}

Manifold* Manifold::Portal::getExitSpace() {
	return exit->getSpace();
}

void Manifold::Portal::setInvert(bool invert) {
	this->invert = invert;
}

bool Manifold::Portal::getInvert() {
	return invert;
}

Manifold::GeodesicPtr Manifold::Portal::teleport(Manifold::GeodesicPtr geodesic) {
	assert(exit);
	//std::cout << "Manifold.cpp teleport from " << getSpace()->getType() << std::endl;
	IntersectionPtr intersection = getIntersection(geodesic);
	if(rotate) {
		intersection->rotate(rotation);
	}
	if(invert ^ exit->getInvert()) {
		intersection->invert();
	}
	return exit->getGeodesic(intersection);
}

Manifold::PointPtr Manifold::Portal::teleport(Manifold::PointPtr point) {
	assert(exit);
	//assert(point->isInManifold());
	return exit->getPoint(getTransport(point));
}

Manifold::GeodesicPtr Manifold::nextPiece(Manifold::GeodesicPtr previous) {
	assert(this == previous->getSpace());
	//std::cout << "Manifold.cpp Space:\t" << getType() << std::endl;
	/*std::cout << "Manifold.cpp Length:\t" << previous->getVector().norm() << std::endl;*/
	//assert(previous->getStart()->getPosition()->isInManifold());
	/*if(getType() == "SurfaceOfRevolution<PortalSpace2d>") {
		std::cout << "Manifold.cpp t before:\t" << ((SurfaceOfRevolution<PortalSpace2d>::Point*) previous->getStart()->getPosition().get())->getT() << std::endl;
	}*/
	double distance = previous->getVector().norm();
	int portal = -1;
	for(int i=0; i<portals.size(); ++i) {
		double current = previous->intersectionDistance(portals[i]);
		//std::cout << "Manifold.cpp current:\t" << current << std::endl;
		//std::cout << "Manifold.cpp portal:\t\t" << i << std::endl;
		assert(current > 0);
		if(current < distance) {
			distance = current;
			portal = i;
		}
	}
	if(portal == -1) {
		/*if(getType() == "SurfaceOfRevolution<PortalSpace2d>") {
			std::cout << "Manifold.cpp t after:\t" << ((SurfaceOfRevolution<PortalSpace2d>::Point*) previous->getEndPoint().get())->getT() << std::endl;
		}*/
		//std::cout << "Manifold.cpp " << getType() << std::endl;
		assert(previous->getEndPoint()->isInManifold());
		return Manifold::GeodesicPtr();
	} else {
		Manifold::GeodesicPtr next = portals[portal]->teleport(previous);
		//std::cout << "teleport\nportal:\t" << portal << "\ndistance:\t" << distance << std::endl;
		return next;
	}
}

void Manifold::addPortal(Manifold::PortalPtr portal) {
	portals.push_back(portal);
}

std::vector<Manifold::PortalPtr>* Manifold::getPortals() {
	return &portals;
}

