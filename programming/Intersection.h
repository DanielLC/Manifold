#ifndef intersection_h
#define intersection_h

#include <tr1/memory>
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

class Intersection {
	public:
		Vector3d getPosition();
		Matrix3d getOrientation();
		Vector3d getVector();
		Intersection(Vector3d position, Matrix3d orientation, Vector3d vector);
		void rotate(Matrix3d rotation);
		void invert();
		bool getSign();
	private:
		Vector3d position;
		Matrix3d orientation;
		Vector3d vector;
};

typedef std::tr1::shared_ptr<Intersection> IntersectionPtr;

#endif
