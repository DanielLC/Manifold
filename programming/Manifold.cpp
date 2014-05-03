#include "Manifold.h"
#include "Euclidean.h"
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

Manifold* Manifold::PointOfReference::getSpace() {
	return getPosition()->getSpace();
}

Manifold* Manifold::Geodesic::getSpace() {
	return getEndPoint()->getSpace();
}

Manifold* Manifold::Portal::getSpace() {
	return space;
}

void Manifold::Portal::setExit(Manifold::Portal* exit) {
	this->exit = exit;
}

void Manifold::Portal::setSpace(Manifold* space) {
	this->space = space;
}

Manifold::GeodesicPtr Manifold::Portal::teleport(Manifold::GeodesicPtr geodesic) {
	assert(exit);
	//std::cout << "Manifold::Portal::teleport() error:\t" << (this->getGeodesic(getIntersection(geodesic))->getEndPoint()->getVector() -geodesic->getEndPoint()->getVector()).squaredNorm() << std::endl;
	//std::cout << "Manifold::Portal::teleport() error:\n" << this->getGeodesic(getIntersection(geodesic))->getEndPoint()->getVector() -geodesic->getEndPoint()->getVector() << std::endl;
	//std::cout << "Original length:\t" << geodesic->getVector().norm() << std::endl;
	//std::cout << "Remaining distance:\t" << getGeodesic(getIntersection(geodesic))->getVector().norm() << std::endl;
	assert((getGeodesic(getIntersection(geodesic))->getEndPoint()->getVector() - geodesic->getEndPoint()->getVector()).squaredNorm() < EPSILON*EPSILON);
	//std::cout << "Input length:\t" << geodesic->getVector().norm() << std::endl;
	//std::cout << "Output length:\t" << getGeodesic(getIntersection(geodesic))->getVector().norm() << std::endl;
	//assert((exit->getGeodesic(getIntersection(geodesic))->getEndPoint()->getVector() - geodesic->getEndPoint()->getVector()).squaredNorm() < EPSILON);
	//std::cout << exit->getGeodesic(getIntersection(geodesic))->getEndPoint()->getVector() << std::endl;
	//std::cout << "Manifold::Portal::teleport(Manifold::GeodesicPtr geodesic)" << std::endl;
	return exit->getGeodesic(getIntersection(geodesic));
}

Manifold::GeodesicPtr Manifold::nextPiece(Manifold::GeodesicPtr previous) {
	double distance = previous->getVector().norm();
	int portal = -1;
	for(int i=0; i<portals.size(); ++i) {
		double current = previous->intersectionDistance(portals[i]);
		//std::cout << "current:\t" << current << std::endl;
		//std::cout << "portal:\t\t" << i << std::endl;
		//std::cout << "current:\t" << current << std::endl;
		assert(current > 0);
		if(current < distance) {
			distance = current;
			portal = i;
		}
	}
	if(portal == -1) {
		return Manifold::GeodesicPtr();
	} else {
		Manifold::GeodesicPtr next = portals[portal]->teleport(previous);
		assert((next->getEndPoint()->getVector() - previous->getEndPoint()->getVector()).squaredNorm() < EPSILON); //TODO only works on portals that lead to themselves.
		//std::cout << "teleport\nportal:\t" << portal << "\ndistance:\t" << distance << std::endl;
		return next;
	}
}

void Manifold::addPortal(Manifold::PortalPtr portal) {
	portals.push_back(portal);
}

