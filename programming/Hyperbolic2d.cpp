#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic2d.h"
#include "Assert.h"
#include <Eigen/Dense>
#include <utility>
#include <algorithm>
using Eigen::Vector2d;
using Eigen::Matrix2d;

Hyperbolic2d::Point::Point(double x, double y) {
	assert(y > 0.00000001);
	coordinates[0] = x;
	coordinates[1] = y;
}

Hyperbolic2d::Point::Point() {
	coordinates[1] = 1;
}

std::tr1::array<double,2> Hyperbolic2d::Point::getCoordinates(){return coordinates;}

void Hyperbolic2d::Point::setCoordinates(double x, double y) {
	coordinates[0] = x;
	coordinates[1] = y;
}

Vector2d Hyperbolic2d::Point::vectorFromPoint(Hyperbolic2d::Point point) {
	return this->getGeodesic(point)->getVector();
}

Vector2d Hyperbolic2d::Line::getVector() {
	return z;
}

Vector2d Hyperbolic2d::Circle::getVector() {
	return z;
}

std::tr1::shared_ptr<Hyperbolic2d::Geodesic> Hyperbolic2d::Point::getGeodesic(Hyperbolic2d::Point point) {
	std::tr1::array<double,2> x = this->getCoordinates();
	std::tr1::array<double,2> y = point.getCoordinates();
	assert(x[1] > 0);
	if(fabs(x[0] - y[0]) < 0.00001) {
		return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Line(this, point, Vector2d(0.,log(y[1]/x[1]))));
	}
	//std::cout << "fabs(x[0] - y[0]) >= 0.00001\n";
	double c = ((x[0]*x[0]+x[1]*x[1])-(y[0]*y[0]+y[1]*y[1]))/(2*(x[0]-y[0]));
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dir0 = x[1]/r;
	double dir1 = (c-x[0])/r;
	double dist = log((x[1]*(r+y[0]-c))/(y[1]*(r+x[0]-c)));
	Vector2d z(dir0*dist,dir1*dist);
	/*std::cout << "y:	(" << y[0] << ",	" << y[1] << ")\n";
	std::cout << "c:	" << c << "\n";
	std::cout << "r:	" << r << "\n";
	std::cout << "dir0:	" << dir0 << "\n";
	std::cout << "dir1:	" << dir1 << "\n";
	std::cout << "dist:	" << dist << "\n";
	std::cout << "(r+x[0]-c):	" << (r+x[0]-c) << "\n";
	std::cout << "(r+y[0]-c):	" << (r+y[0]-c) << "\n";
	std::cout << "x[1]:	" << x[1] << "\n";
	std::cout << "y[1]:	" << y[1] << "\n";
	std::cout << "(y[1]*(r+x[0]-c))/(x[1]*(r+y[0]-c)):	" << (y[1]*(r+x[0]-c))/(x[1]*(r+y[0]-c)) << "\n";*/
	assert(z[0] == z[0]);
	assert(z[1] == z[1]);
	/*std::cout << "pointFromVector(out).coordinates[0]:	" << pointFromVector(out).coordinates[0] << "\n";
	std::cout << "pointFromVector(out).coordinates[1]:	" << pointFromVector(out).coordinates[1] << "\n";
	std::cout << "point.coordinates[0]:	" << point.coordinates[0] << "\n";
	std::cout << "point.coordinates[1]:	" << point.coordinates[1] << "\n";*/
	//assert(fabs(this->pointFromVector(z).getCoordinates()[0] - point.getCoordinates()[0]) < 0.000001);
	//assert(fabs(this->pointFromVector(z).getCoordinates()[1] - point.getCoordinates()[1]) < 0.000001);
	return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Circle(this, point, z, c, r));
}

Hyperbolic2d::Point Hyperbolic2d::Point::pointFromVector(Vector2d z) {
	return this->getGeodesic(z)->getEndPoint();
}

std::tr1::shared_ptr<Hyperbolic2d::Geodesic> Hyperbolic2d::Point::getGeodesic(Vector2d z) {
	assert(z == z);
	std::tr1::array<double,2> x = this->getCoordinates();
	assert(x[1] > EPSILON);
	if(fabs(z[0]) < EPSILON) {
		return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Line(this, Hyperbolic2d::Point(x[0],x[1]*exp(z[1])), z));
	}
	//std::cout << "fabs(z[0]) >= 0.000001\n";
	double c = x[0]+x[1]*z[1]/z[0];
	double r = sqrt(x[1]*x[1]+(x[0]-c)*(x[0]-c));
	double dist = sqrt(z[0]*z[0]+z[1]*z[1]);
	if(z[0] > 0) {
		dist *= -1;
		//std::cout << "z[0] > 0\n";
	} else {
		//std::cout << "z[0] <= 0\n";
	}
	double rx0c2 = r+x[0]-c;
	rx0c2 *= rx0c2;
	double x12e2d = x[1]*x[1]*exp(2*dist);
	double y0 = ((r+c)*rx0c2-x12e2d*(r-c))/(x12e2d+rx0c2);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	//std::cout << "z:	(" << z[0] << ",	" << z[1] << ")" << std::endl;
	//std::cout << "|z|:\t" << sqrt(z[0]*z[0]+z[1]*z[1]) << std::endl;
	//std::cout << "y1:\t" << y1 << std::endl;
	assert(y1 > 0.00000001);
	/*std::cout << "point:	(" << x[0] << ",	" << x[1] << ")\n";
	std::cout << "2d vector:	(" << z[0] << ",	" << z[1] << ")\n";
	std::cout << "2d point:	(" << y0 << ",	" << y1 << ")\n";
	Vector2d difference = this->getGeodesic(Hyperbolic2d::Point(y0,y1))->getVector();
	std::cout << "difference:	(" << difference[0] << ",	" << difference[1] << ")\n";*/
	/*std::cout << "z:\n" << z << "\n";
	std::cout << "c:	" << c << "\n";
	std::cout << "r:	" << r << "\n";*/
	//assert(fabs(log((y1*(r+x[0]-c))/(x[1]*(r+y0-c))) - dist) < 0.000001);
	Hyperbolic2d::Point end(y0,y1);
	return std::tr1::shared_ptr<Hyperbolic2d::Geodesic>(new Hyperbolic2d::Circle(this, end, z, c, r));
}

Hyperbolic2d::Point Hyperbolic2d::Line::getStartPoint() {
	return start;
}

Hyperbolic2d::Point Hyperbolic2d::Circle::getStartPoint() {
	return start;
}

Hyperbolic2d::Point Hyperbolic2d::Line::getEndPoint() {
	return end;
}

Hyperbolic2d::Point Hyperbolic2d::Circle::getEndPoint() {
	return end;
}

std::pair<Hyperbolic2d::Point, double> Hyperbolic2d::Point::pointAndRotFromVector(Vector2d z) {
	std::tr1::shared_ptr<Hyperbolic2d::Geodesic> geodesic = this->getGeodesic(z);
	return std::pair<Hyperbolic2d::Point, double>(geodesic->getEndPoint(), geodesic->getRot());
}

double Hyperbolic2d::Line::getRot() {
	return 0;
}

double Hyperbolic2d::Circle::getRot() {
	std::tr1::array<double,2> x = start.getCoordinates();
	std::tr1::array<double,2> y = end.getCoordinates();
	double angle0 = atan2(x[1],x[0]-c);	//This is pointing towards the center. Since I'm just looking at the difference, it doesn't really matter.
	double angle1 = atan2(y[1],y[0]-c);
	//std::cout << "2d:" << std::endl;
	//std::cout << "x[1]:	" << x[1] << std::endl;
	//std::cout << "x[0]-c:	" << x[0]-c << "\nangle0:	" << angle0 << "\nangle1:	" << angle1 << std::endl;
	//This could be optimized by taking the arccos of the dot product.
	return angle1-angle0;
}

double Hyperbolic2d::Line::wormholeIntersectionDistance(double portal) {
	double edist = (start.getCoordinates()[1])/(start.getCoordinates()[0]*portal);
	if(edist < EPSILON) {
		return INFINITY;
	}
	double dist = log(edist);
	if(z[1] < 0) {
		dist = -dist;
	}
	if(dist < EPSILON) {
		return INFINITY;
	} else {
		return dist;
	}
}

double Hyperbolic2d::Circle::wormholeIntersectionDistance(double portal) {
	if(c > r) {
		if((c > 0) ^ (portal < 0)) {
			return INFINITY;
		}
	}
	double a2 = portal*portal/(portal*portal+1);
	double b2 = c*c-a2;
	double d2 = r*r-b2;
	if(d2 < EPSILON) {
		return INFINITY;
	}
	double a = sqrt(a2);
	double d = sqrt(d2);
	std::tr1::array<double,2> x = start.getCoordinates();
	Vector2d dir;
	dir << portal, 1;
	dir.normalize();
	Vector2d y0 = (a-d)*dir;
	double dist = INFINITY;
	if(y0[1] > 0 && ((z[0] > 0) ^ (y0[0] < x[0]))) {
		dist = fabs(log((x[1]*(r+y0[0]-c))/(y0[1]*(r+x[0]-c))));
	}
	Vector2d y1 = (a+d)*dir;
	if(y1[1] > 0 && ((z[0] > 0) ^ (y1[0] < x[0]))) {
		dist = std::min(dist,fabs(log((x[1]*(r+y1[0]-c))/(y1[1]*(r+x[0]-c)))));
	}
	assert(dist > 0);
	return dist;
}

Intersection2d Hyperbolic2d::Line::wormholeGetIntersection(double portal) {
	double dist = wormholeIntersectionDistance(portal);
	Vector2d vector;
	vector << 1, portal;
	vector.normalize();
	if(z[1] > 0) {
		vector *= z[1]-dist;
	} else {
		vector *= z[1]+dist;
	}
	return Intersection2d(log(start.getCoordinates()[1]),0,vector);
}

Intersection2d Hyperbolic2d::Circle::wormholeGetIntersection(double portal) {
	double a2 = portal*portal/(portal*portal+1);
	double b2 = c*c-a2;
	double d2 = r*r-b2;
	double a = sqrt(a2);
	double d = sqrt(d2);
	std::tr1::array<double,2> x = start.getCoordinates();
	Vector2d dir;
	dir << portal, 1;
	dir.normalize();
	Vector2d y;
	Vector2d y0 = (a-d)*dir;
	double dist = INFINITY;
	bool flag;
	if((z[0] > 0) ^ (y0[0] < x[0])) {
		dist = log((x[1]*(r+y0[0]-c))/(y0[1]*(r+x[0]-c)));
		y = y0;
		//d = a-d;	//Redefining d
		flag = false;
	}
	Vector2d y1 = (a+d)*dir;
	if((z[0] > 0) ^ (y1[0] < x[0])) {
		double dist2 = log((x[1]*(r+y1[0]-c))/(y1[1]*(r+x[0]-c)));
		if(dist2 < dist) {
			dist = dist2;
			y = y1;
			//d = a+d;	//Redefining d
			flag = true;
		}
	}
	
	/*double cos = (d*d+r*r-c*c)/(2*d*r);
	double sin = sqrt(1-cos*cos);
	if(c < 0) {
		sin = -sin;
	}
	Vector2d vector(dist*sin,dist*cos); //I mixed up the two curves again.*/
	
	
	/*double dist = wormholeIntersectionDistance(portal);
	Vector2d dir(portal,1);
	dir.normalize();*/
	double sin = asin((dir[0]*c*dir-(Vector2d() << c,0).finished()).norm()/r);
	double cos = sqrt(1-sin);
	Vector2d vector = (z.norm()-dist)*Vector2d(sin,cos);
	if(flag) {
		vector[1] *= -1;
	}
	double rot = asin(y[0]-c)-asin(start.getCoordinates()[0]-c) - (atan(y[0]/y[1]) - atan(start.getCoordinates()[0]/start.getCoordinates()[1]));
	Intersection2d intersection(log(y.squaredNorm())/2,rot,vector);
	#ifndef NDEBUG
	Hyperbolic2d::GeodesicPtr geodesic = wormholeGetGeodesic(intersection, portal);
	assert(abs(geodesic->getStartPoint().getCoordinates()[0]-start.getCoordinates()[0]) < EPSILON);
	assert(abs(geodesic->getStartPoint().getCoordinates()[1]-start.getCoordinates()[1]) < EPSILON);
	#endif
	return intersection;
}

Hyperbolic2d::Circle::Circle(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z, double c, double r) {
	this->start = *start;
	this->end = end;
	this->c = c;
	this->r = r;
	this->z = z;
}

Hyperbolic2d::Line::Line(Hyperbolic2d::Point* start, Hyperbolic2d::Point end, Vector2d z) {
	this->start = *start;
	this->end = end;
	this->z = z;
	assert(fabs(z[0]) < EPSILON);
}

Hyperbolic2d::GeodesicPtr Hyperbolic2d::wormholeGetGeodesic(Intersection2d intersection, double portal) {
	Matrix2d rot;
	rot << portal,1,-1,portal;
	rot /= sqrt(portal*portal+1);
	Vector2d vector = rot*intersection.getVector();
	Vector2d dir;
	dir << portal, 1;
	dir.normalize();
	dir *= intersection.getPosition();
	Hyperbolic2d::Point start(dir[0],dir[1]);
	return start.getGeodesic(vector);
}

