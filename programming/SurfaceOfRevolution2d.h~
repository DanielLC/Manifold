#ifndef SurfaceOfRevolution2d_h
#define SurfaceOfRevolution2d_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>

using Eigen::Vector2d;

class SurfaceOfRevolution2d {
	public:
		class Point {
			public:
				virtual ~Point();
				virtual double getT() = 0;
				virtual double getThetaK() = 0;
				virtual std::tr1::array<double,2> getCoordinates() = 0;
				virtual void setCoordinates(double t, double thetak) = 0;
				virtual Vector2d vectorFromPoint(Point point) = 0;
				virtual Point pointFromVector(Vector2d vector) = 0;
				virtual std::pair<Point, double> pointAndRotFromVector(Vector2d vector) = 0;
		};
};
#endif

