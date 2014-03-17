#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Hyperbolic.h"
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

Hyperbolic::Point::Point() {
	coordinates[2] = 1;
}

//std::tr1::array<double,3> Hyperbolic::Point::coordinates;
Hyperbolic::Point::Point(double x, double y, double z, Hyperbolic* space) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	this->space = space;
}
//Hyperbolic::Point::~Point() {}

Hyperbolic* Hyperbolic::Point::getSpace() {
	return space;
}

std::tr1::array<double,3> Hyperbolic::Point::getCoordinates(){return coordinates;}

//Hyperbolic::Point Hyperbolic::PointOfReference::position;
//Matrix3d Hyperbolic::PointOfReference::orientation;

/*Manifold::Point Hyperbolic::Point::midpoint(Manifold::Point point) {
	
}*/

Vector3d Hyperbolic::PointOfReference::vectorFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {
	std::tr1::array<double,3> x = position.coordinates;
	std::tr1::array<double,3> y = std::tr1::static_pointer_cast<Hyperbolic::Point>(point)->coordinates;
	if(x[0] == y[0] && x[1] == y[1]) {
		return orientation.transpose()*Vector3d(x[0],x[1],log(y[2]/x[2]));
	}
	//std::cout << "x: " << x[0] << "," << x[1] << "," << x[2] << "\n";
	//std::cout << "y: " << y[0] << "," << y[1] << "," << y[2] << "\n";
	double v0 = y[0]-x[0];
	double v1 = y[1]-x[1];
	double x1 = x[2];
	double y0 = sqrt(v0*v0+v1*v1);
	double y1 = y[2];
	//std::cout << "y_0: " << y0 << "\n";
	v0 /= y0;
	v1 /= y0;
	double c = ((y0*y0+y1*y1)-x1*x1)/(2*y0);
	double r = sqrt(x1*x1+c*c);
	double dir0 = x1/r;
	double dir1 = c/r;
	double dist = log((y1*(r-c))/(x1*(r+y0-c)));
	Vector3d out(v0*dir0*dist,v1*dir0*dist,dir1*dist);
	//std::cout << (c - x1*(dir1*dist)/(dir0*dist)) << "\n";
	//std::cout << ((c-y0)*(c-y0)+y1*y1-r*r) << "\n";
	/*double rc = r-c;
	double x12e2d = x1*x1*exp(2*dist);
	std::cout << ((r+c)*rc-x12e2d)*rc/(x12e2d+rc*rc)-y0 << "\n";*/
	out = orientation.transpose()*out;
	return out;

/*	double v[] = {y[0]-x[0],y[1]-x[1]};
	double y3 = sqrt(v[0]*v[0]+v[1]*v[1]);
	v[0] /= y3;
	v[1] /= y3;
	double c = (y3*y3+y[2]*y[2]-x[2]*x[2])/(2*y3);
	double r = sqrt(x[2]*x[2]+c*c);
	double dir[] = {x[2]/r,c/r};
	double dist = log((y[2]*(r-c))/(x[2]*(r+y3-c)));
	//std::cout << y[2] << "," << (r-c) << "," << x[2] << "," << (r+y3-c) << "\n";
	//std::cout << dir[0] << "," << dir[1] << "," << dist << "\n\n";
	Vector3d out(v[0]*dir[0]*dist,v[1]*dir[0]*dist,dir[1]*dist);
	out *= orientation;
	return out;*/
}

std::vector<Vector3d> Hyperbolic::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {
	std::vector<Vector3d> out;
	out.push_back(this->vectorFromPoint(point));
	return out;
}

std::tr1::shared_ptr<Manifold::Point> Hyperbolic::PointOfReference::pointFromVector(Vector3d vector) {
	vector = orientation*vector;
	std::tr1::array<double,3> x = position.coordinates;
	if(vector[0] == 0 && vector[1] == 0) {
		return std::tr1::shared_ptr<Manifold::Point>(new Hyperbolic::Point(x[0],x[1],x[2]*exp(vector[2]),position.space));
	}
	double x1 = x[2];
	double z0 = sqrt(vector[0]*vector[0]+vector[1]*vector[1]);
	double z1 = vector[2];
	double v0 = vector[0]/z0;
	double v1 = vector[1]/z0;
	double c = x1*z1/z0;
	double r = sqrt(x1*x1+c*c);
	double dist = sqrt(z0*z0+z1*z1);
	double rc = r-c;
	double x12e2d = x1*x1*exp(2*dist);
	double y0 = ((r+c)*rc-x12e2d)*rc/(x12e2d+rc*rc);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	std::cout << "Point:	" << "(" << x[0]+y[0]*v0 << "",	"" << x[1]+y[0]*v1 << "",	"" << y[1] << ")\n";
	return std::tr1::shared_ptr<Manifold::Point>(new Hyperbolic::Point(x[0]+y0*v0,x[1]+y0*v1,y1,position.space));
}

//Hyperbolic::Hyperbolic() {}
//Hyperbolic::~Hyperbolic() {}

/*Manifold::Triangle Hyperbolic::makeTriangle() {
	std::tr1::shared_ptr<Hyperbolic::Point> vertex0(new Hyperbolic::Point(1,0,2));
	std::tr1::shared_ptr<Hyperbolic::Point> vertex1(new Hyperbolic::Point(0,1,1));
	std::tr1::shared_ptr<Hyperbolic::Point> vertex2(new Hyperbolic::Point(1,0,1));
	Hyperbolic::Triangle triangle = {vertex0,vertex1,vertex2};
	return triangle;
}*/

void Hyperbolic::Point::setCoordinates(double x, double y, double z) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
}

Hyperbolic::PointOfReference::PointOfReference(Hyperbolic* space) {
	orientation = orientation.Identity();
	position.space = space;
}

/*Hyperbolic::PointOfReference::~PointOfReference() {
}*/

Manifold::Point* Hyperbolic::PointOfReference::getPosition() {
	return &position;
}

void Hyperbolic::PointOfReference::move(Vector3d vector) {
	//std::cout << vector << "\n";
	//std::cout << orientation << "\n";
	vector = orientation*vector;
	std::tr1::array<double,3> x = position.coordinates;
	//std::cout << x[0] << "," << x[1] << "," << x[2] << "\n";
	if(vector[0] == 0 && vector[1] == 0) {
		position.coordinates[2] *= exp(vector[2]);
		return;
		//return std::tr1::shared_ptr<Manifold::Point>(new Hyperbolic::Point(x[0],x[1],x[2]*exp(vector[2])));
	}

	double x1 = x[2];
	double z0 = sqrt(vector[0]*vector[0]+vector[1]*vector[1]);
	double z1 = vector[2];
	double v0 = vector[0]/z0;
	double v1 = vector[1]/z0;
	//std::cout << "x_1: " << x1 <<"\n";
	//std::cout << "z: " << z0 << "," << z1 <<"\n";
	double c = x1*z1/z0;	//It really looks like I have this backwards, but only with w/s.
	/*std::cout << "c: " << c <<"\n";
	std::cout << "v: " << v0 << "," << v1 <<"\n";
	std::cout << "center: " << c*v0+x[0] << "," << c*v1+x[1] << "\n";*/
	double r = sqrt(x1*x1+c*c);
	//std::cout << r << "\n";
	double dist = sqrt(z0*z0+z1*z1);
	double rc = r-c;
	double x12e2d = x1*x1*exp(2*dist);
	double y0 = ((r+c)*rc-x12e2d)*rc/(x12e2d+rc*rc);
	//double y0 = ((c*exp(2*dist) - r*exp(2*dist))*x1*x1 + c*c*c - c*c*r - c*r*r + r*r*r)/(x1*x1*exp(2*dist) + c*c - 2*c*r + r*r);
	double y1 = sqrt(r*r-(y0-c)*(y0-c));
	//std::cout << y0 << "," << y1 << "\n";
	/*std::cout << vector[0] << "," << vector[1] << "," << vector[2] << "\n";
	Vector3d test = vectorFromPoint(std::tr1::shared_ptr<Manifold::Point>(new Hyperbolic::Point(x[0]+y0*v0,x[1]+y0*v1,y1)));
	std::cout << test[0] << "," << test[1] << "," << test[2] << "\n";*/
	//std::cout << v0 << "," << v1 << "," << y0 << "," << y1 << "," << c << "\n";
	//std::cout << dist << "\n";
	//std::cout << log((y1*(r-c))/(x1*(r+y0-c))) << "\n";
	/*std::cout << "forward: " << vector;
	Vector3d down(0,0,-r);
	down = orientation*down;
	std::cout << down;
	Matrix3d m;
	m << v0,v1,0,-v1,v0,0,0,0,1;
	down = m*down;
	std::cout << down;
	down = Matrix3d((y0-c)/r,0,-y1/r,0,1,0,y1/r,0,(y0-c)/r)*down;
	//down.print();
	down = Matrix3d(-c/r,0,x1/r,0,1,0,-x1/r,0,-c/r)*down;
	std::cout << down;
	down = Matrix3d(v0,-v1,0,v1,v0,0,0,0,1)*down;
	std::cout << down;*/
	position.setCoordinates(x[0]+y0*v0,x[1]+y0*v1,y1);
	//std::cout << position.coordinates[0] << "," << position.coordinates[1] << "," << position.coordinates[2] << "\n";
	orientation = (Matrix3d() << v0,v1,0,-v1,v0,0,0,0,1).finished()*orientation;
	/*Vector3d test(vector[0],vector[1],vector[2]);
	test *= Matrix3d(v0,v1,0,-v1,v0,0,0,0,1);
	std::cout << test[0] << "," << test[1] << "," << test[2] << "\n";*/
	/*test *= Matrix3d(v0,-v1,0,v1,v0,0,0,0,1);
	std::cout << test[0] << "," << test[1] << "," << test[2] << "\n";*/
	/*Matrix3d test((y0-c)/r,0,-y1/r,0,1,0,y1/r,0,(y0-c)/r);
	test.premult(Matrix3d(-c/r,0,x1/r,0,1,0,-x1/r,0,-c/r));
	test.print();*/
	orientation = (Matrix3d() << (y0-c)/r,0,-y1/r,0,1,0,y1/r,0,(y0-c)/r).finished()*orientation;
	orientation = (Matrix3d() << -c/r,0,x1/r,0,1,0,-x1/r,0,-c/r).finished()*orientation;
	/*double theta0 = atan2(x1,-c);
	double theta1 = atan2(y1,y0-c);
	double co = cos(theta1-theta0);
	double si = sin(theta1-theta0);
	orientation.premult(Matrix3d(co,0,-si,0,1,0,si,0,co));*/
	orientation = (Matrix3d() << v0,-v1,0,v1,v0,0,0,0,1).finished()*orientation;
	/*down = Vector3d << 0,0,-r;
	down = orientation*down;
	std::cout << down;*///Before, orientation*down was not getting the same answer as multiplying by all the matrices in turn.
	//std::cout << position.coordinates[0]+down[0] << "," << position.coordinates[1]+down[1] << "," << position.coordinates[2]+down[2] << "\n";
	/*Vector3d down2(0,0,-r);
	down2 *= Matrix3d(0,(y0-c)/r,-y1/r,0,1,0,y1/r,0,(y0-c)/r);
	down2 *= Matrix3d(0,-c/r,x1/r,0,1,0,-x1/r,0,-c/r);*/
	//std::cout << down[0] << "," << down[1] << "," << down[2] << "\n";
	//std::cout << position.coordinates[0] << "," << position.coordinates[1] << "," << position.coordinates[2] << "\n";
	//std::cout << position.coordinates[0]+down2[0] << "," << position.coordinates[1]+down2[1] << "," << position.coordinates[2]+down2[2] << "\n";
	//orientation /= orientation.determinate();	//I need to divide by the cube root of that.
	/*std::tr1::shared_ptr<Matrix3d> m = orientation.transpose();
	m->premult(orientation);
	m->print();*/

	//std::cout << orientation.determinate() << "\n";
}

void Hyperbolic::PointOfReference::rotate(Matrix3d rot) {
	orientation = rot*orientation;
}

