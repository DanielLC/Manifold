#ifndef Cylinder2d_h
#define Cylinder2d_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>

using Eigen::Vector2d;

class Cylinder2d {
	public:
		class Point {
			public:
				Point(double t, double thetak);
				Point();
				double getT();
				double getThetaK();
				std::tr1::array<double,2> getCoordinates();
				void setCoordinates(double t, double thetak);
				Vector2d vectorFromPoint(Cylinder2d::Point point);
				Cylinder2d::Point pointFromVector(Vector2d vector);
				std::pair<Point, double> pointAndRotFromVector(Vector2d vector);
			private:
				std::tr1::array<double,2> coordinates;
		};
};
#endif

