#ifndef Compound_h
#define Compound_h

#include <vector>
#include <tr1/array>
#include <tr1/memory>
#include "Manifold.h"
#include <Eigen/Core>
using Eigen::Vector3d;
using Eigen::Matrix3d;

class Compound : public Manifold {
	public:
		class Point;
		class PointOfReference;
		class Geodesic;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		class Point {
			public:
				Compound* getSpace();
				Vector3d getVector();
				Point(std::tr1::shared_ptr<Manifold::Point> position);
				Point();
				Manifold* getSubspace();
				std::tr1::shared_ptr<Manifold::Point> getPosition();
			private:
				std::tr1::shared_ptr<Manifold::Point> position;
				Compound* space;
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference);
				Vector3d vectorFromPointAndNearVector(std::tr1::shared_ptr<Compound::Point> point, Vector3d vector);
				//std::tr1::shared_ptr<Point> pointFromVector(Vector3d vector);
				Manifold::Point* getPosition();
				void rotate(Matrix3d rot);
			private:
				std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference;
		};
		class Geodesic : public Manifold::Geodesic {
			public:
				Geodesic(PointOfReferencePtr start, PointOfReferencePtr end, Vector3d vector);
				PointPtr getEndPoint();
				PointOfReferencePtr getEndPointOfReference();
				Vector3d getVector();
			private:
				PointOfReferencePtr start;
				
		};
		GeodesicPtr getGeodesic(PointOfReferencePtr start, PointPtr end);
		GeodesicPtr getGeodesic(PointOfReferencePtr start, Vector3d vector);
};
#endif
