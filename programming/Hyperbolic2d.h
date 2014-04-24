#ifndef Hyperbolic2d_h
#define Hyperbolic2d_h

#include <tr1/array>
#include <tr1/memory>
#include <Eigen/Core>
#include <utility>
#include "Intersection2d.h"

using Eigen::Vector2d;

class Hyperbolic2d {
	public:
		class Geodesic;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		class Point {
			public:
				Point(double x, double y);
				Point();
				std::tr1::array<double,2> getCoordinates();
				void setCoordinates(double x, double y);
				Vector2d vectorFromPoint(Point point);
				Point pointFromVector(Vector2d z);
				std::pair<Point, double> pointAndRotFromVector(Vector2d z);
				GeodesicPtr getGeodesic(Vector2d z);
				GeodesicPtr getGeodesic(Point end);
			private:
				std::tr1::array<double,2> coordinates;
		};
		class Geodesic {
			public:
				virtual Point getStartPoint() = 0;
				virtual Point getEndPoint() = 0;
				virtual double getRot() = 0;
				virtual Vector2d getVector() = 0;
				virtual double wormholeIntersectionDistance(double portal) = 0;
				virtual Intersection2d wormholeGetIntersection(double portal) = 0;
		};
		class Circle : public Geodesic {
			public:
				Circle(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z, double c, double r);
				Point getStartPoint();
				Point getEndPoint();
				double getRot();
				Vector2d getVector();
				double wormholeIntersectionDistance(double portal);
				Intersection2d wormholeGetIntersection(double portal);
			private:
				Hyperbolic2d::Point start;
				Hyperbolic2d::Point end;
				Vector2d z;
				double c;
				double r;
		};
		class Line : public Geodesic {
			public:
				Line(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z);
				Point getStartPoint();
				Point getEndPoint();
				double getRot();
				Vector2d getVector();
				double wormholeIntersectionDistance(double portal);
				Intersection2d wormholeGetIntersection(double portal);
			private:
				Hyperbolic2d::Point start;
				Hyperbolic2d::Point end;
				Vector2d z;
		};
		static GeodesicPtr wormholeGetGeodesic(Intersection2d intersection, double portal);
};
#endif

