#ifndef PortalSpace2d2_h
#define PortalSpace2d2_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>
#include "Hyperbolic2d.h"
using Eigen::Vector2d;

class PortalSpace2d {
	public:
		class Point {
			public:
				Point(Hyperbolic2d::Point hyperbolic);
				Point(double t, double thetak);
				Point();
				double getSin();
				double getCos();
				double getT();
				double getThetaK();
				std::tr1::array<double,2> getCoordinates();
				void setCoordinates(double t, double thetak);
				Vector2d vectorFromPoint(PortalSpace2d::Point point);
				PortalSpace2d::Point pointFromVector(Vector2d vector);
				std::pair<Point, double> pointAndRotFromVector(Vector2d vector);
			private:
				Hyperbolic2d::Point hyperbolic;
		};
};
#endif

