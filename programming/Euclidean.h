#ifndef euclidean_h
#define euclidean_h

#include <vector>
#include <tr1/array>
#include <tr1/memory>
#include "Manifold.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

class Euclidean : public Manifold {
	public:
		class Point;
		class PointOfReference;
		class Geodesic;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		class Point : public Manifold::Point {
			public:
				Euclidean* getSpace();
				Point(double x, double y, double z, Euclidean* space);
				Point(Vector3d coordinates, Euclidean* space);
				Point(Euclidean* space);
				Point();
				Vector3d getCoordinates();
				Vector3d getVector();
				void setCoordinates(double x, double y, double z);
			private:
				Vector3d coordinates;
				Euclidean* space;
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(PointPtr position, Matrix3d orientation);
				PointOfReference(Euclidean* space);
				Manifold::PointPtr getPosition();
				//void setPosition(Manifold::Point* position);
				void rotate(Matrix3d rot);
				Matrix3d getOrientation();
				Vector3d getCoordinates();
			private:
				PointPtr position;
				Matrix3d orientation;
		};
		class Geodesic : public Manifold::Geodesic {
			public:
				Geodesic(PointOfReferencePtr start, PointPtr end, Vector3d vector);
				Manifold::PointPtr getEndPoint();
				Manifold::PointOfReferencePtr getEndPointOfReference();
				Vector3d getVector();
			private:
				PointOfReferencePtr start;
				PointPtr end;
				Vector3d vector;
		};
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector);
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end);
};
#endif

