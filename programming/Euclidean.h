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
		class PointOfReference;
		class Point : public Manifold::Point {
			public:
				Euclidean* getSpace();
				Point(double x, double y, double z, Euclidean* space);
				Point();
				friend class PointOfReference;
				//Point midpoint(Point point);
			private:
				std::tr1::array<double,3> coordinates;
				Euclidean* space;
		};
		class PointOfReference : public Manifold::PointOfReference {		//Add manifold
			public:
				PointOfReference(Euclidean* space);
				~PointOfReference();
				std::vector<Vector3d> vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point);
				std::tr1::shared_ptr<Manifold::Point> pointFromVector(Vector3d vector);
				Manifold::Point* getPosition();
				//void setPosition(Manifold::Point* position);
				void move(Vector3d dir);
				void rotate(Matrix3d rot);
			private:
				Point position;
				Matrix3d orientation;
		};
//		typedef std::tr1::array<std::tr1::shared_ptr<Manifold::Point>,3> Triangle;

		Triangle makeTriangle();		//For debug purposes
//		std::vector<Triangle> triforce(Triangle);
};
#endif

