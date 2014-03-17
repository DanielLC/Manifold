#ifndef Hyperbolic_h
#define Hyperbolic_h

#include <tr1/array>
#include <tr1/memory>
#include "Manifold.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

class Hyperbolic : public Manifold {
	public:
		class Point;
		class PointOfReference;
		class Geodesic;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		class Point : public Manifold::Point {
			public:
				Manifold* getSpace();
				Point(double x, double y, double z, Hyperbolic* space);
				Point(Hyperbolic* space);
				Point();
				std::tr1::array<double,3> getCoordinates();
				void setCoordinates(double x, double y, double z);
				Vector3d getVector();
				//Point midpoint(Point point);
			private:
				std::tr1::array<double,3> coordinates;
				Hyperbolic* space;
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(PointPtr position, Matrix3d orientation);
				PointOfReference(Hyperbolic* space);
				Manifold::PointPtr getPosition();
				void rotate(Matrix3d rot);
				Matrix3d getOrientation();
			private:
				PointPtr position;
				Matrix3d orientation;
		};
		class Geodesic : public Manifold::Geodesic {
			public:
				Geodesic(Manifold::PointOfReferencePtr start, std::tr1::shared_ptr<Point> end, Vector3d vector, double v0, double v1, std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic2d);
				Manifold::PointPtr getEndPoint();
				Manifold::PointOfReferencePtr getEndPointOfReference();
				Vector3d getVector();
			private:
				Manifold::PointOfReferencePtr start;
				Manifold::PointPtr end;
				Vector3d vector;
				double v0;
				double v1;
				std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic2d;
		};
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector);
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end);
		/*Vector3d vectorFromPoint(Manifold::PointOfReferencePtr start, Manifold::PointPtr end);
		std::vector<Vector3d> vectorsFromPoint(Manifold::PointOfReferencePtr start, Manifold::PointPtr end);
		Manifold::PointPtr pointFromVector(Manifold::PointOfReferencePtr start, Vector3d vector);*/
		Manifold::PointOfReferencePtr getPointOfReference(Manifold::PointOfReferencePtr start, Vector3d vector);
//		typedef std::tr1::array<std::tr1::shared_ptr<Manifold::Point>,3> Triangle;

		//Triangle makeTriangle();		//For debug purposes
//		std::vector<Triangle> triforce(Triangle);
};
#endif

