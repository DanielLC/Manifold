#include "Manifold.h"

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

