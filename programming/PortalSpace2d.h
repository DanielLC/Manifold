#ifndef PortalSpace2d_h
#define PortalSpace2d_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>
#include "Hyperbolic2d.h"
using Eigen::Vector2d;

class PortalSpace2d {
	public:
		class Geodesic;
		class Point {
			public:
				Point(Hyperbolic2d::Point hyperbolic);
				Point(double t, double thetak);
				Point();
				double getT();
				double getThetaK();
				std::tr1::array<double,2> getCoordinates();
				void setCoordinates(double x, double y);
				Vector2d vectorFromPoint(PortalSpace2d::Point point);
				PortalSpace2d::Point pointFromVector(Vector2d vector);
				std::pair<Point, double> pointAndRotFromVector(Vector2d vector);
				Hyperbolic2d::Point getPosition();
				std::tr1::shared_ptr<Geodesic> getGeodesic(Vector2d z);
				std::tr1::shared_ptr<Geodesic> getGeodesic(Point end);
				double operator[](int i);
			private:
				Hyperbolic2d::Point hyperbolic;
		};
		class Geodesic {
			public:
				Geodesic(std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic);
				Point getStartPoint();
				Point getEndPoint();
				double getRot();
				Vector2d getVector();
			private:
				std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic;
		};
};
#endif

