#ifndef manifold_h
#define manifold_h

#include <tr1/memory>
#include <vector>
#include <tr1/array>
#include <Eigen/Core>
#include <string>
#include "Intersection.h"
#include "PointTransport.h"
using Eigen::Matrix3d;
using Eigen::Vector3d;

//TODO: It might be possible to make it a template, given the address of a constant pointer to k.
class Manifold {
	public:
		class Point;
		class PointOfReference;
		class Geodesic;
		class Portal;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		typedef std::tr1::shared_ptr<Portal> PortalPtr;
		typedef std::tr1::array<PointPtr,3> Triangle;
		class Point {
			public:
				bool isInManifold();
				virtual Manifold* getSpace() = 0;
				virtual Vector3d getVector() = 0;
		};
		class PointOfReference {
			public:
				virtual void rotate(Matrix3d rot) = 0;
				virtual PointPtr getPosition() = 0;
				Manifold* getSpace();
		};
		class Geodesic {
			public:
				virtual PointPtr getEndPoint() = 0;
				virtual PointOfReferencePtr getEndPointOfReference() = 0;
				virtual PointOfReferencePtr getStart() = 0;
				virtual Vector3d getVector() = 0;
				Manifold* getSpace();
				
				virtual double intersectionDistance(PortalPtr portal) = 0;
		};
		class Portal {		//Currently, this only supports spheres. It could be generalized to generalized spheres. This would require another kind of intersection.
			public:
				Manifold* getSpace();
				GeodesicPtr teleport(GeodesicPtr geodesic);
				PointPtr teleport(PointPtr point);
				void setExit(Portal* exit);
				void setSpace(Manifold* space);
				Manifold* getExitSpace();
				virtual bool containsPoint(Manifold::Point* point) = 0;
				virtual double getRadiusOfCurvature() = 0;
				virtual double getCircumference() = 0;
			private:
				virtual GeodesicPtr getGeodesic(IntersectionPtr intersection) = 0;
				virtual IntersectionPtr getIntersection(GeodesicPtr geodesic) = 0;
				virtual Manifold::PointPtr getPoint(PointTransportPtr transport) = 0;
				virtual PointTransportPtr getTransport(Manifold::PointPtr point) = 0;
				Portal* exit;
				Manifold* space;
		};
		virtual std::string getType() = 0;
		Vector3d vectorFromPoint(PointOfReferencePtr start, PointPtr end);		//Maybe these two should be in PointOfReference.
		PointPtr pointFromVector(PointOfReferencePtr start, Vector3d vector);	//
		PointOfReferencePtr getPointOfReference(PointOfReferencePtr start, Vector3d vector);
		virtual GeodesicPtr getGeodesic(PointOfReferencePtr start, PointPtr end) = 0;
		virtual GeodesicPtr getGeodesic(PointOfReferencePtr start, Vector3d vector) = 0;
		std::vector<Triangle> icosahedron(PointOfReferencePtr por);
		std::vector<Triangle> icosahedron(PointOfReferencePtr por, double k);
		std::vector<Triangle> octahedron(PointOfReferencePtr por);
		std::vector<Triangle> octahedron(PointOfReferencePtr por, double k);
		//virtual Triangle makeTriangle() = 0;		//For debug purposes
		//virtual std::vector<Triangle> makeTriangleList() = 0;
		
		GeodesicPtr nextPiece(GeodesicPtr previous);
		void addPortal(PortalPtr portal);
		std::vector<PortalPtr>* getPortals();
	private:
		std::vector<PortalPtr> portals;
};

#endif
