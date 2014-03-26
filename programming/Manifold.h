#ifndef manifold_h
#define manifold_h

#include <tr1/memory>
#include <vector>
#include <tr1/array>
#include <Eigen/Core>
#include "Intersection.h"
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
				virtual Vector3d getVector() = 0;
				Manifold* getSpace();
				
				virtual double intersectionDistance(PortalPtr portal) = 0;
		};
		class Portal {		//Currently, this only supports spheres. It could be generalized to generalized spheres. This would require another kind of intersection.
			public:
				Manifold* getSpace();
				GeodesicPtr teleport(GeodesicPtr geodesic);
				void setExit(Portal* exit);
				void setSpace(Manifold* space);
				//virtual Portal* getExit();
			private:
				virtual GeodesicPtr getGeodesic(IntersectionPtr intersection) = 0;
				virtual IntersectionPtr getIntersection(GeodesicPtr geodesic) = 0;
				Portal* exit;
				Manifold* space;
		};
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
	private:
		std::vector<PortalPtr> portals;
};

#endif
