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
		class Point {
			public:
				virtual Manifold* getSpace() = 0;
				virtual ~Point();
		};
		typedef std::tr1::shared_ptr<Point> PointP;
		typedef std::tr1::array<PointP,3> Triangle;
		class PointOfReference {
			public:
				virtual ~PointOfReference();
				virtual std::vector<Vector3d> vectorsFromPoint(std::tr1::shared_ptr<Point> point) = 0;
				virtual std::tr1::shared_ptr<Point> pointFromVector(Vector3d vector) = 0;
				virtual void move(Vector3d dir) = 0;
				virtual void rotate(Matrix3d rot) = 0;
				virtual Point* getPosition() = 0;
				//virtual void setPosition(Manifold::Point* position) = 0;
				std::vector<Triangle> icosahedron();
				std::vector<Triangle> icosahedron(double k);
				std::vector<Triangle> octahedron();
				std::vector<Triangle> octahedron(double k);
		};
		virtual ~Manifold();

		//virtual Triangle makeTriangle() = 0;		//For debug purposes
		//virtual std::vector<Triangle> makeTriangleList() = 0;
};

#endif
