#ifndef SurfaceOfRevolution_h
#define SurfaceOfRevolution_h

#include <vector>
#include <tr1/array>
#include <tr1/memory>
#include <iostream>
#include "Manifold.h"
#include "SurfaceOfRevolution.h"
#include <Eigen/Core>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Assert.h"
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;

template <class SurfaceOfRevolution2d>
class SurfaceOfRevolution : public Manifold {
	public:
		double getK();
		class PointOfReference;
		class Point : public Manifold::Point {
			public:
				SurfaceOfRevolution* getSpace();
				Point(double t, double x, double y, double z, SurfaceOfRevolution* space);	//x,y, and z give the position on S^2, and w gives the height on the surface of revolution. I can afford an extra degree of freedom. i tells how many times you have to loop around the sphere.
				Point();
				friend class PointOfReference;
			private:
				Vector3d spherical;
				double t;
				SurfaceOfRevolution* space;
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(SurfaceOfRevolution* space);
				~PointOfReference();
				std::vector<Vector3d> vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point);
				std::tr1::shared_ptr<Manifold::Point> pointFromVector(Vector3d vector);
				Manifold::Point* getPosition();
				void move(Vector3d dir);
				void rotate(Matrix3d rot);
			private:
				Point position;
				Matrix4d orientation;	//Orientation * relative vector = absolute vector.
		};
//		typedef std::tr1::array<std::tr1::shared_ptr<Manifold::Point>,3> Triangle;

		Triangle makeTriangle();		//For debug purposes
//		std::vector<Triangle> triforce(Triangle);
};

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::getK() {
	//return log(2*M_PI)/(2*M_PI);
	return 1;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::Point() {
	spherical << 0,0,1;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::Point(double t, double x, double y, double z, SurfaceOfRevolution* space) {
		//t is a monotonic function of the non-spherical direction. It doesn't have to be any particular function.
	this->t = t;
	spherical[0] = x;
	spherical[1] = y;
	spherical[2] = z;
	this->space = space;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>* SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::getSpace() {
	return space;
}

template <class SurfaceOfRevolution2d>
std::vector<Vector3d> SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {	//Don't forget to check if they're on the same vertical line. Also, show more points.
	assert(point->getSpace() == position.space);
	double k = position.space->getK();
	Vector3d x = position.spherical;
	std::tr1::shared_ptr<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point> p = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(point);
	if(fabs(p->t - position.t) < EPSILON && ((p->spherical - position.spherical).norm() < EPSILON) || ((p->spherical + position.spherical).norm() < EPSILON)) {
		//Change that to squared norm.
		//There are other vectors that work, but they're in literally every direction. It's a "shortest path between North and South pole" kind of thing.
		std::vector<Vector3d> outVector;
		outVector.push_back(Vector3d(0,0,0));
		return outVector;
	}
	Vector3d y = p->spherical;
	assert(fabs(x.norm()-1) < EPSILON);
	assert(y == y);
	assert(fabs(y.norm()-1) < EPSILON);
	double dot = x.dot(y);
	Vector3d yy = y-dot*x;
	yy.normalize();			//yy is the normalization of the component of y perpendicular to x. This makes it the spherical component of the direction towards y.
	while(yy != yy) {
		Vector3d rand;
		rand.Random();
		dot = x.dot(rand);
		yy = rand-dot*x;
	}
	double theta = acos(dot);
	typename SurfaceOfRevolution2d::Point x2d(position.t,0.);
	typename SurfaceOfRevolution2d::Point y2d(p->t,theta*k);
	Vector2d z = x2d.vectorFromPoint(y2d);
	//std::cout << "theta*k:\n" << theta*k << "\n";
	//std::cout << "z:\n" << z << "\n";
	Vector4d out;
	out << z[0], z[1]*yy;
	out = orientation.transpose()*out;
	assert(abs(out[3]) < EPSILON);
	std::vector<Vector3d> outVector;
	//std::cout << "out: " << out << "\n" << std::flush;
	outVector.push_back(out.start<3>());
	/*std::cout << "std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->t:	" << std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->t << "\n";
	std::cout << "p->t:	" << p->t << "\n";
	std::cout << "outVector[0]:\n" << outVector[0] << "\n";
	std::cout << "std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->spherical:\n" << std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->spherical << "\n";
	std::cout << "p->spherical:\n" << p->spherical << "\n";*/
	assert(fabs(std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->t - p->t) < EPSILON);
	assert((std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->spherical - p->spherical).norm() < EPSILON);
	return outVector;
}

template <class SurfaceOfRevolution2d>
std::tr1::shared_ptr<Manifold::Point> SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::pointFromVector(Vector3d vector) {
	assert(vector == vector);
	double k = position.space->getK();
	Vector4d v;
	v << vector, 0;
	v = orientation*v;
	typename SurfaceOfRevolution2d::Point x2d(position.t,0.);
	Vector2d z2d(v[0],v.end<3>().norm());
	assert(z2d == z2d);
	std::tr1::array<double,2> y2d = x2d.pointFromVector(z2d).getCoordinates();
	y2d[1] /= k;
	Vector4d y;
	Vector3d ytail;
	if(y2d[1] == 0) {
		ytail = position.spherical;
	} else {
		ytail = position.spherical*cos(y2d[1]/k)+v.end<3>().normalized()*sin(y2d[1]/k);
	}
	y << y2d[0], ytail;
	assert(y == y);
	std::tr1::shared_ptr<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point> out = std::tr1::shared_ptr<SurfaceOfRevolution::Point>(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(y[0],y[1],y[2],y[3],position.space));
	//std::cout << "out->t:	" << out->t << "\n";
	return out;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::PointOfReference(SurfaceOfRevolution<SurfaceOfRevolution2d>* space) {
	orientation = orientation.Identity();
	position.space = space;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::~PointOfReference() {
}

template <class SurfaceOfRevolution2d>
Manifold::Point* SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::getPosition() {
	return &position;
}

template <class SurfaceOfRevolution2d>
void SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::move(Vector3d dir) {	//Don't forget to check if they're on the same vertical line.
	double k = position.space->getK();
	Vector4d v;
	v << dir, 0;
	v = orientation*v;
	typename SurfaceOfRevolution2d::Point x2d(position.t,0.);
	Vector2d z2d(v[0],v.end<3>().norm());
	std::pair<typename SurfaceOfRevolution2d::Point, double> pointAndRot = x2d.pointAndRotFromVector(z2d);
	std::tr1::array<double,2> y2d = pointAndRot.first.getCoordinates();
	y2d[1] /= k;
	double rot = pointAndRot.second;
	//Crashes if v.end<3>() = 0;
	Vector3d row1 = v.end<3>().normalized();
	Vector3d row2 = position.spherical;
	if(row1 != row1) {
		row1 = Vector3d().Random();
		row1 -= row2*row2.dot(row1);
		row1.normalize();
	}
	Vector3d row3 = row1.cross(row2);
	Vector3d ytail = position.spherical*cos(y2d[1]/k)+row1*sin(y2d[1]/k);
	//std::cout << "ytail.norm()-1:	" << ytail.norm()-1 << "\n";
	assert(fabs(ytail.norm()-1) < EPSILON);
	/*std::cout << "v.end<3>():\n" << v.end<3>() << "\n";
	std::cout << "row1:\n" << row1 << "\n";	
	std::cout << "position.spherical:\n" << position.spherical << "\n";
	std::cout << "row2:\n" << row2 << "\n";
	std::cout << "row1:\n" << row1 << "\n" << std::flush;
	std::cout << "row2:\n" << row2 << "\n" << std::flush
	std::cout << "row3:\n" << row3 << "\n" << std::flush;
	//std::cout << "|row1 x row2|:	" << row1.cross(row2).norm() << "\n" << std::flush;
	std::cout << "row1 . row2:\n" << row1.dot(row2) << "\n" << std::flush;
	std::cout << "row2 . row3:\n" << row2.dot(row3) << "\n" << std::flush;
	std::cout << "row3 . row1:\n" << row3.dot(row1) << "\n" << std::flush;*/
	Matrix4d conjugate;
	/*conjugate << 1,0,0,0,
		0,row1,
		0,row2,
		0,row3;*/
	conjugate.block<4,1>(0,0) << 1,0,0,0;	//Why can't I just use rows? Or just make the whole matrix?
	conjugate.block<4,1>(0,1) << 0,row1;
	conjugate.block<4,1>(0,2) << 0,row2;
	conjugate.block<4,1>(0,3) << 0,row3;
	//std::cout << "conjugate:\n" << conjugate << "\n" << std::flush;
	assert((conjugate*conjugate.transpose()-Matrix4d::Identity()).norm() < EPSILON);	//Error is here.
	orientation = conjugate.transpose()*orientation;
	//std::cout << "orientation:\n" << orientation << "\n" << std::flush;

	Matrix4d m = Matrix4d::Identity();
	m.block<2,2>(0,0) << cos(rot),sin(rot),-sin(rot),cos(rot);
	assert(fabs(m.determinant()-1) < EPSILON);
	assert((m*m.transpose()-Matrix4d::Identity()).norm() < EPSILON);
	orientation = m*orientation;

	double sine = sin(y2d[1]/k);
	double cosine = cos(y2d[1]/k);
	m = Matrix4d::Identity();
	m.block<2,2>(1,1) << cosine,-sine,sine,cosine;
	assert(fabs(m.determinant()-1) < EPSILON);
	assert((m*m.transpose()-Matrix4d::Identity()).norm() < EPSILON);
	orientation = m.transpose()*orientation;	//TODO: Am I supposed to do that?

	orientation = conjugate*orientation;
	//std::cout << "(orientation*orientation.transpose()-Matrix4d::Identity()).norm():	" << (orientation*orientation.transpose()-Matrix4d::Identity()).norm() << "\n";
	assert((orientation*orientation.transpose()-Matrix4d::Identity()).norm() < EPSILON);	//There seems to be a numeric instability here. It doesn't need to be corrected often, but I need a good way to check when it does. I don't think the correction is fast.
	orientation += orientation.transpose().inverse();
	orientation /= 2;
	//Since the inverse and transpose are supposed to be equal, averaging it with the inverse of the transpose corrects the error somewhat and keeps it from building up.
	
	position.t = y2d[0];
	position.spherical = ytail;	//TODO: I'm going to need to rearrange this stuff.
	
	/*std::cout << "position.spherical:\n" << position.spherical << "\n";
	std::cout << "orientation*Vector4d(0,0,0,1):\n" << orientation*Vector4d(0,0,0,1) << "\n";
	std::cout << "(orientation.transpose()*(Vector4d() << 0, position.spherical).finished()).start(3).norm():\n" << (orientation.transpose()*(Vector4d() << 0, position.spherical).finished()).start(3).norm() << "\n";*/
	assert((orientation.transpose()*(Vector4d() << 0, position.spherical).finished()).start(3).norm() < EPSILON);
}

template <class SurfaceOfRevolution2d>
void SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::rotate(Matrix3d rot) {
	orientation.block<3,3>(0,0) = rot*orientation.block<3,3>(0,0);
}
#endif

