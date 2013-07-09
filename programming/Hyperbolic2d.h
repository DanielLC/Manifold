#ifndef Hyperbolic2d_h
#define Hyperbolic2d_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>

using Eigen::Vector2d;

class Hyperbolic2d {
	public:
		class Point {
			public:
			
				Point(double x, double y);
				Point();
				std::tr1::array<double,2> getCoordinates();
				void setCoordinates(double x, double y);
				Vector2d vectorFromPoint(Point point);
				Point pointFromVector(Vector2d z);
				std::pair<Point, double> pointAndRotFromVector(Vector2d z);
			private:
				std::tr1::array<double,2> coordinates;
		};
};
#endif

