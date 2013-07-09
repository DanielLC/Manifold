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
		class PointOfReference;
		class Point : public Manifold::Point {
			public:
				Hyperbolic* getSpace();
				Point(double x, double y, double z, Hyperbolic* space);
				Point();
				std::tr1::array<double,3> getCoordinates();
				void setCoordinates(double x, double y, double z);
				friend class PointOfReference;
				//Point midpoint(Point point);
			private:
				std::tr1::array<double,3> coordinates;
				Hyperbolic* space;
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(Hyperbolic* space);
				~PointOfReference();
				Vector3d vectorFromPoint(std::tr1::shared_ptr<Manifold::Point> point);
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

		//Triangle makeTriangle();		//For debug purposes
//		std::vector<Triangle> triforce(Triangle);
};
#endif

