#ifndef Compound_h
#define Compound_h

#include <vector>
#include <tr1/array>
#include <tr1/memory>
#include "Manifold.h"
#include <Eigen/Core>
using Eigen::Vector3d;
using Eigen::Matrix3d;

class Compound {
	public:
		class PointOfReference;
		class Point {
			public:
				//Manifold* getSpace();
				//Vector3d getVector();
				Point(std::tr1::shared_ptr<Manifold::Point> position);
				Point();
				Manifold* getSubspace();
				std::tr1::shared_ptr<Manifold::Point> getPosition();
				friend class PointOfReference;
			private:
				std::tr1::shared_ptr<Manifold::Point> position;
		};
		typedef std::tr1::shared_ptr<Point> PointP;
		typedef std::tr1::array<PointP,3> Triangle;
		class PointOfReference {
			public:
				PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference);
				~PointOfReference();
				//std::vector<Vector3d> vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point);
				Vector3d vectorFromPointAndNearVector(std::tr1::shared_ptr<Compound::Point> point, Vector3d vector);
				std::tr1::shared_ptr<Point> pointFromVector(Vector3d vector);
				Manifold::Point* getPosition();
				//void setPosition(Manifold::Point* position);
				void move(Vector3d dir);
				void rotate(Matrix3d rot);
				
				std::vector<Triangle> icosahedron();
				std::vector<Triangle> icosahedron(double k);
				std::vector<Triangle> octahedron();
				std::vector<Triangle> octahedron(double k);
			private:
				//Point position;
				//Matrix3d orientation;
				std::tr1::shared_ptr<Manifold::PointOfReference> pointOfReference;
		};
//		typedef std::tr1::array<std::tr1::shared_ptr<Manifold::Point>,3> Triangle;

//		Triangle makeTriangle();		//For debug purposes
//		std::vector<Triangle> triforce(Triangle);
};
#endif

