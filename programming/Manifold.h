#ifndef manifold_h
#define manifold_h

#include <tr1/memory>
#include <vector>
#include <tr1/array>
#include <Eigen/Dense>
using Eigen::Matrix3d;
using Eigen::Vector3d;

//TODO: It might be possible to make it a template, given the address of a constant pointer to k.
class Manifold {
	public:
		class Point;
		class PointOfReference;
		class Geodesic;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
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
};

#endif
