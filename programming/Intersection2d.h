#ifndef intersection2d_h
#define intersection2d_h

#include <vector>
#include <tr1/memory>
#include <Eigen/Core>
using Eigen::Vector2d;

//TODO: It might be possible to make it a template, given the address of a constant pointer to k.
class Intersection2d {
		public:
			Intersection2d(double position, double rotation, Vector2d vector);
			double getPosition();
			double getRotation();
			Vector2d getVector();
		private:
			double position;
			double rotation;
			Vector2d vector;
};

typedef std::tr1::shared_ptr<Intersection2d> Intersection2dPtr;

#endif
