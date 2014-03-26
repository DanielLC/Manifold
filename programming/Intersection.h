#ifndef intersection_h
#define intersection_h

#include <vector>
#include <tr1/memory>
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

//TODO: It might be possible to make it a template, given the address of a constant pointer to k.
class Intersection {
	public:
		Vector3d getPosition();
		Matrix3d getOrientation();
		Vector3d getVector();
		Intersection(Vector3d position, Matrix3d orientation, Vector3d vector);
		void rotate(Matrix3d rotation);
		void invert();
	private:
		Vector3d position;
		Matrix3d orientation;
		Vector3d vector;
};

typedef std::tr1::shared_ptr<Intersection> IntersectionPtr;

#endif
