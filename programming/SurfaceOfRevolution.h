#ifndef SurfaceOfRevolution_h
#define SurfaceOfRevolution_h

#include <vector>
#include <tr1/array>
#include <tr1/memory>
#include <iostream>
#include "Manifold.h"
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Assert.h"
using Eigen::Matrix;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;

template <class SurfaceOfRevolution2d>
class SurfaceOfRevolution : public Manifold {
	public:
		double getK();								//the circumference of the bottleneck = log(2*pi*k)
		class Point;
		class PointOfReference;
		class Geodesic;
		typedef std::tr1::shared_ptr<Point> PointPtr;
		typedef std::tr1::shared_ptr<PointOfReference> PointOfReferencePtr;
		typedef std::tr1::shared_ptr<Geodesic> GeodesicPtr;
		class Point : public Manifold::Point {
			public:
				SurfaceOfRevolution* getSpace();	//Gives the space this point is in
				void setSpace(SurfaceOfRevolution* space);
				Point(double t, double x, double y, double z, SurfaceOfRevolution* space);	//x,y, and z give the position on S^2, and t gives the height on the surface of revolution. I can afford an extra degree of freedom.
				Point(SurfaceOfRevolution* space);
				Point();
				Vector3d getSpherical();			//Gives the S^2 coordinate
				double getT();						//Gives the R coordinate
				Vector3d getVector();
			private:
				Vector3d spherical;					//S^2 coordinate
				double t;							//R coordinate. In this case, it's in the interval (0, pi)
				SurfaceOfRevolution* space;			//The space the point is in
		};
		class PointOfReference : public Manifold::PointOfReference {
			public:
				PointOfReference(SurfaceOfRevolution* space);	//Gives a default point in that space
				PointOfReference(PointPtr position, Matrix4d orientation);
				//~PointOfReference();
				//std::vector<Vector3d> vectorsFromPoint(Manifold::PointPtr point);
				//Manifold::PointPtr pointFromVector(Vector3d vector);
				Manifold::PointPtr getPosition();
				Matrix4d getOrientation();
				//void move(Vector3d dir);			//Moves in the direction of dir for the distance equal to its magnituded
				void rotate(Matrix3d rot);			//Rotates the orientation
			private:
				PointPtr position;
				Matrix4d orientation;				//Orientation * relative vector = absolute vector.
		};
		class Geodesic : public Manifold::Geodesic {
			public:
				//Only should be constructed by SurfaceOfRevolution. Don't worry about the constructor.
				Geodesic(PointOfReferencePtr start, PointPtr end, Vector3d vector, std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d);
				Manifold::PointPtr getEndPoint();
				Manifold::PointOfReferencePtr getEndPointOfReference();	//Gives the point of reference you'd have after moving through the geodesic without rotating.
				Vector3d getVector();				//Direction and distance from start
				std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> getGeodesic2d();
				Manifold::PointOfReferencePtr getStart();

				double intersectionDistance(Manifold::PortalPtr portal);
				double intersectionDistance2(Manifold::Portal* portal);
			private:
				PointOfReferencePtr start;
				PointPtr end;
				Vector3d vector;
				std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d;	//Corresponding 2d geodesic
		};
		class Portal : public Manifold::Portal {
			public:
				Portal(bool side, Manifold* space);	//For attaching to Euclidean geometry. This is intended just for <Portalspace2d>
				Portal(double t, Manifold* space);
				double getT();
			private:
				double t;
				Manifold::GeodesicPtr getGeodesic(IntersectionPtr intersection);
				IntersectionPtr getIntersection(Manifold::GeodesicPtr geodesic);
		};
		//These are how you actually get geodesics.
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector);			//Gives the geodesic that starts at start and has the direction and distance of vector
		Manifold::GeodesicPtr getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end);		//Gives a geodesic that starts at start and ends at end
};

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::getK() {
	//return exp(2*M_PI)/(2*M_PI);
	return 1;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::Point() {
	spherical << 0,0,1;
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::Point(SurfaceOfRevolution* space) {
	spherical << 0,0,1;
	this->space = space;
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
void SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::setSpace(SurfaceOfRevolution<SurfaceOfRevolution2d>* space) {
	this->space = space;
}

template <class SurfaceOfRevolution2d>
Vector3d SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::getSpherical() {
	return spherical;
}

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::getT() {
	return t;
}

template <class SurfaceOfRevolution2d>
Vector3d SurfaceOfRevolution<SurfaceOfRevolution2d>::Point::getVector() {
	//std::cout << "spherical:\n" << spherical << std::endl;
	//std::cout << "t:\t" << t << std::endl;
	return spherical*exp(t/256);
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::PointOfReference(SurfaceOfRevolution<SurfaceOfRevolution2d>* space) {
	orientation = Matrix4d::Identity();
	/*Matrix3d randrot;		//Randomizes the rotation.
	do {
		randrot.setRandom();
		while((randrot*randrot.transpose()-Matrix3d::Identity()).norm() > EPSILON) {
			randrot += randrot.inverse().transpose();
			randrot /= 2;
		}
	} while(randrot.determinant() <= 0);
	orientation.block<3,3>(0,0) = randrot;
	//std::cout << "orientation:\n" << orientation << "\n";*/
	position = SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(space));
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::PointOfReference(SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr position, Matrix4d orientation) {
	this->orientation = orientation;
	this->position = position;
}

/*template <class SurfaceOfRevolution2d>
std::vector<Vector3d> SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::vectorsFromPoint(std::tr1::shared_ptr<Manifold::Point> point) {	//Don't forget to check if they're on the same vertical line. Also, show more points.
	assert(point->getSpace() == position->space);
	double k = position->space->getK();
	Vector3d x = position->spherical;
	std::tr1::shared_ptr<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point> p = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(point);
	if(fabs(p->t - position->t) < EPSILON && ((p->spherical - position->spherical).norm() < EPSILON) || ((p->spherical + position->spherical).norm() < EPSILON)) {
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
	Vector3d yy;
	if(fabs(dot) == 1) {		//If yy ends up undefined, presumably because x and y have the same or opposite position in spherical geometry. It doesn't matter what value you use.
		Vector3d rand;
		rand.setRandom();
		//std::cout << "rand:\n" << rand << "\n";
		double dot2 = x.dot(rand);
		yy = rand-dot2*x;
		//std::cout << "yy:\n" << yy << "\n";
	} else {
		yy = y-dot*x;
	}
	yy.normalize();			//yy is the normalization of the component of y perpendicular to x. This makes it the spherical component of the direction towards y.
	assert(yy == yy);

	double theta = acos(dot);
	typename SurfaceOfRevolution2d::Point x2d(position->t,0.);
	//std::cout << "p->t:	" << p->t << "\n";
	//std::cout << "theta*k:	" << theta*k << "\n";
	typename SurfaceOfRevolution2d::Point y2d(p->t,theta*k);
	Vector2d z2d = x2d.vectorFromPoint(y2d);
	//std::cout << "z2d:\n" << z2d << "\n";
	//std::cout << "theta*k:\n" << theta*k << "\n";
	//std::cout << "z:\n" << z << "\n";
	Vector4d out;
	out << z2d[0], z2d[1]*yy;
	out = orientation.transpose()*out;
	assert(out == out);
	assert(abs(out[3]) < EPSILON);
	std::vector<Vector3d> outVector;
	//std::cout << "out: " << out << "\n" << std::flush;
	outVector.push_back(out.start<3>());
	assert(fabs(std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->t - p->t) < EPSILON);
	//std::cout << "position->t:	" << position->t << "\n";
	assert((std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(this->pointFromVector(outVector[0]))->spherical - p->spherical).norm() < EPSILON);
	
	return outVector;
}*/

/*template <class SurfaceOfRevolution2d>
std::tr1::shared_ptr<Manifold::Point> SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::pointFromVector(Vector3d vector) {
	assert(fabs(position->spherical.norm()-1) < EPSILON);
	assert(vector == vector);
	double k = position->space->getK();
	Vector4d v;
	v << vector, 0;
	v = orientation*v;
	assert(fabs(position->spherical.dot(v.end<3>())) < EPSILON);
	//std::cout << "v:\n" << v << "\n";
	typename SurfaceOfRevolution2d::Point x2d(position->t,0.);
	Vector2d z2d(v[0],v.end<3>().norm());
	assert(z2d == z2d);
	std::tr1::array<double,2> y2d = x2d.pointFromVector(z2d).getCoordinates();
	Vector4d y;
	Vector3d ytail;
	if(y2d[1] == 0) {
		ytail = position->spherical;
	} else {
		ytail = position->spherical*cos(y2d[1]/k)+v.end<3>().normalized()*sin(y2d[1]/k);
		//If yd2[1] is zero, then v.end<3>() is zero, so normalizing it gives nan. Multiplying by zero doesn't make this go away.
	}
	y << y2d[0], ytail;
	assert(y == y);
	assert(fabs(ytail.norm()-1) < EPSILON);	//TODO:Assertion failed.
	std::tr1::shared_ptr<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point> out = std::tr1::shared_ptr<SurfaceOfRevolution::Point>(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(y[0],y[1],y[2],y[3],position->space));
	//std::cout << "out->t:	" << out->t << "\n";
	//assert((vectorsFromPoint(out)[0]-vector).norm() < EPSILON);
	return out;
}*/

template <class SurfaceOfRevolution2d>
Manifold::PointPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::getPosition() {
	return std::tr1::static_pointer_cast<Manifold::Point>(position);
}

template <class SurfaceOfRevolution2d>
Matrix4d SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::getOrientation() {
	return orientation;
}

template <class SurfaceOfRevolution2d>
Manifold::PointOfReferencePtr SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::getEndPointOfReference() {
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr s =
				std::tr1::static_pointer_cast<typename SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition());
	Vector3d spherical = s->getSpherical();
	Matrix4d orientation = start->getOrientation();
	//std::cout << "Before:\n" << orientation << std::endl;
	assert((orientation.transpose()*(Vector4d() << 0, spherical).finished()).start(3).norm() < EPSILON);
	double k = ((SurfaceOfRevolution<SurfaceOfRevolution2d>*) start->getSpace())->getK();
	Vector4d v;
	v << vector, 0;
	v = orientation*v;
	//std::cout << "v.end<3>().norm():	" << v.end<3>().norm() << "\n";
	//std::cout << "v.end<3>().dot(position->spherical):	" << v.end<3>().dot(position->spherical) << "\n";
	assert(fabs(v.end<3>().dot(spherical)) < EPSILON);
	double rot = geodesic2d->getRot();
	//Crashes if v.end<3>() = 0;
	Vector3d row2 = spherical;
	Vector3d row1;
	assert(fabs(row2.norm()-1) < EPSILON);
	if(v.end<3>().norm() < EPSILON) {
		row1 = Vector3d().setRandom();
		row1 -= row2*row2.dot(row1);
		row1.normalize();
		//std::cout << "row1 randomized" << std::endl;
		//std::cout << "row1 = nan\n";
	} else {
		row1 = v.end<3>().normalized();
		//std::cout << "v:\n" << v;
	}
	//std::cout << "row1.dot(row2):	" << row1.dot(row2) << "\n";
	assert(fabs(row1.dot(row2)) < EPSILON);
	Vector3d row3 = row1.cross(row2);
	Matrix4d conjugate;
	conjugate.block<4,1>(0,0) << 1,0,0,0;	//Why can't I just use rows? Or just make the whole matrix?
	conjugate.block<4,1>(0,1) << 0,row1;	//These are columns, not rows.
	conjugate.block<4,1>(0,2) << 0,row2;
	conjugate.block<4,1>(0,3) << 0,row3;
	//std::cout << "conjugate:\n" << conjugate << "\n";
	//std::cout << "conjugate*conjugate.transpose():\n" << conjugate*conjugate.transpose() << "\n";
	//std::cout << "(conjugate*conjugate.transpose()-Matrix4d::Identity()).norm():	" << (conjugate*conjugate.transpose()-Matrix4d::Identity()).norm() << "\n";
	//std::cout << "conjugate:\n" << conjugate << std::endl;
	assert((conjugate*conjugate.transpose()-Matrix4d::Identity()).norm() < EPSILON);
	orientation = conjugate.transpose()*orientation;
	//std::cout << "orientation:\n" << orientation << "\n" << std::flush;

	Matrix4d m = Matrix4d::Identity();
	m.block<2,2>(0,0) << cos(rot),sin(rot),-sin(rot),cos(rot);
	assert(fabs(m.determinant()-1) < EPSILON);
	assert((m*m.transpose()-Matrix4d::Identity()).norm() < EPSILON);
	//std::cout << "rot:\t" << rot << std::endl;
	//std::cout << "m:\n" << m << std::endl;
	orientation = m*orientation;

	typename SurfaceOfRevolution2d::Point y2d = geodesic2d->getEndPoint();
	//std::cout << "y2d[1]/k:\t" << y2d[1]/k << std::endl;
	double sine = sin(y2d[1]/k);
	double cosine = cos(y2d[1]/k);
	m = Matrix4d::Identity();
	m.block<2,2>(1,1) << cosine,-sine,sine,cosine;
	assert(fabs(m.determinant()-1) < EPSILON);
	assert((m*m.transpose()-Matrix4d::Identity()).norm() < EPSILON);
	orientation = m.transpose()*orientation;

	orientation = conjugate*orientation;
	//std::cout << "Error:\n" << (orientation*orientation.transpose()-Matrix4d::Identity()).norm() << std::endl;
	assert((orientation*orientation.transpose()-Matrix4d::Identity()).squaredNorm() < EPSILON);	//There seems to be a numeric instability here. It doesn't need to be corrected often, but I need a good way to check when it does. I don't think the correction is fast.
	//Since the inverse and transpose are supposed to be equal, averaging it with the inverse of the transpose corrects the error somewhat and keeps it from building up.
	orientation += orientation.transpose().inverse();
	orientation /= 2;

	/*std::cout << "Orientation:\n" << orientation << std::endl;
	std::cout << "position.spherical:\n" << end->getSpherical() << std::endl;
	std::cout << "position.t:\t" << end->getT() << std::endl;*/
	assert(end->getSpherical() == end->getSpherical());
	assert((orientation.transpose()*(Vector4d() << 0, end->getSpherical()).finished()).start(3).norm() < EPSILON);
	//Asserts that a vector aimed perpendicular to reality gets back-transformed to the w coordinate (perpendicular to reality)
	return Manifold::PointOfReferencePtr(new SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference(end, orientation));
}

//TODO: Bug here
//Are you sure? I thought this was working.
//I suppose it doesn't really matter much. I'm not actually using this code in the current form of the program.
template <class SurfaceOfRevolution2d>
Manifold::GeodesicPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::getGeodesic(Manifold::PointOfReferencePtr start, Manifold::PointPtr end) {
	assert(start->getSpace() == this);
	assert(end->getSpace() == this);
	double k = this->getK();
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReferencePtr castedStart =
				std::tr1::static_pointer_cast<typename SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference>(start);
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr s =
				std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition());
	Vector3d x = s->getSpherical();
	std::tr1::shared_ptr<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point> e = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(end);
	if(fabs(e->getT() - s->getT()) < EPSILON && ((e->getSpherical() - s->getSpherical()).norm() < EPSILON) || ((e->getSpherical() + s->getSpherical()).norm() < EPSILON)) {
		//Change that to squared norm.
		//There are other vectors that work, but they're in literally every direction. It's a "shortest path between North and South pole" kind of thing.
		//I really should be making two cases for this. One for if it's on the other side.
		typename SurfaceOfRevolution2d::Point x2d(s->getT(),0.0);
		std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d = x2d.getGeodesic(Vector2d(0.0,0.0));
		return Manifold::GeodesicPtr(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic(castedStart, e, Vector3d(0,0,0),
					geodesic2d));
	}
	Vector3d y = e->getSpherical();
	assert(fabs(x.norm()-1) < EPSILON);
	assert(y == y);
	assert(fabs(y.norm()-1) < EPSILON);
	double dot = x.dot(y);
	Vector3d yy;
	if(fabs(dot) == 1) {		//If yy ends up undefined, presumably because x and y have the same or opposite position in spherical geometry. It doesn't matter what value you use.
		Vector3d rand;
		rand.setRandom();
		//std::cout << "rand:\n" << rand << "\n";
		double dot2 = x.dot(rand);
		yy = rand-dot2*x;
		//std::cout << "yy:\n" << yy << "\n";
	} else {
		yy = y-dot*x;
	}
	yy.normalize();			//yy is the normalization of the component of y perpendicular to x. This makes it the spherical component of the direction towards y.
	assert(yy == yy);

	double theta = acos(dot);
	typename SurfaceOfRevolution2d::Point x2d(s->getT(),0.0);
	//std::cout << "p->t:	" << p->t << "\n";
	//std::cout << "theta*k:	" << theta*k << "\n";
	typename SurfaceOfRevolution2d::Point y2d(e->getT(),theta*k);
	std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d = x2d.getGeodesic(y2d);
	Vector2d z2d = geodesic2d->getVector();
	//std::cout << "z2d:\n" << z2d << "\n";
	//std::cout << "theta*k:\n" << theta*k << "\n";
	//std::cout << "z:\n" << z << "\n";
	Vector4d out;
	out << z2d[0], z2d[1]*yy;
	out = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference>(start)->getOrientation().transpose()*out;
	assert(out == out);
	assert(abs(out[3]) < EPSILON);
	//std::cout << "out: " << out << "\n" << std::flush;
	//assert(fabs(std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->pointFromVector(outVector[0]))->getT() - p->getT()) < EPSILON);
	//std::cout << "position->t:	" << position->t << "\n";
	//assert((std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->pointFromVector(outVector[0]))->getSpherical() - p->getSpherical()).norm() < EPSILON);
	return Manifold::GeodesicPtr(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic(castedStart, e, out.start<3>(), geodesic2d));
}

template <class SurfaceOfRevolution2d>
Manifold::GeodesicPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::getGeodesic(Manifold::PointOfReferencePtr start, Vector3d vector){
	double k = ((SurfaceOfRevolution<SurfaceOfRevolution2d>*) start->getPosition()->getSpace())->getK();
	Vector4d v;
	v << vector,0;
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReferencePtr castedStart =
				std::tr1::static_pointer_cast<typename SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference>(start);
	//std::cout << "castedStart->getOrientation():\n" << castedStart->getOrientation() << std::endl;
	v = castedStart->getOrientation()*v;
	//std::cout << "v:\n" << v << std::endl;
	//std::cout << "v.end<3>():\n" << v.end<3>() << std::endl;
	//std::cout << "v.end<3>().normalized():\n" << v.end<3>().normalized() << "\n";
	typename SurfaceOfRevolution2d::Point x2d(std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition())->getT(),0.);
	Vector2d v2d(v[0],v.end<3>().norm());
	std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d = x2d.getGeodesic(v2d);
	std::tr1::array<double,2> y2d = geodesic2d->getEndPoint().getCoordinates();
	//std::cout << "y2d:\t(" << y2d[0] << ",\t" << y2d[1] << ")" << std::endl;
	y2d[1] /= k;
	Vector4d y;
	Vector3d ytail;
	if(y2d[1] < EPSILON) {
		ytail = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition())->getSpherical();
		//std::cout << "y2d[1] == 0\n";
	} else {
		ytail = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition())->getSpherical()*cos(y2d[1])+v.end<3>().normalized()*sin(y2d[1]);
		//std::cout << "y2d[1] != 0\n";
		//If yd2[1] is zero, then v.end<3>() is zero, so normalizing it gives nan. Multiplying by zero doesn't make this go away.
	}
	//Vector3d ytail = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(start->getPosition())->getSpherical()*cos(y2d[1])+v.end<3>().normalized()*sin(y2d[1]);
	//TODO
	y << y2d[0], ytail;
	/*std::cout << "start:\n" << start->getPosition()->getVector() << std::endl;
	std::cout << "vector:\n" << vector << std::endl;
	std::cout << "end:\n" << ytail*exp(y[0]) << std::endl;*/
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr end(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(y[0],y[1],y[2],y[3],(SurfaceOfRevolution<SurfaceOfRevolution2d>*)start->getSpace()));
	//std::cout << this->vectorsFromPoint(out)[0] << "\n" << vector << "\n" << std::flush; //TODO
	/*std::cout << "pointFromVector:\n";	//TODO
	std::cout << "theta:	" << theta << "\n";
	std::cout << "sin(theta):	" << sin(theta) << "\n";
	std::cout << "x0:	" << x0 << "\nx1:	" << x1 << "\n";
	std::cout << "y0:	" << y0 << "\ny1:	" << y1 << "\n";
	std::cout << "z0:	" << z0 << "\nz1:	" << z1 << "\n";
	std::cout << "c:	" << c << "\nr:	" << r << "\n";
	Vector4d x;
	x << position->hyperbolic, position->spherical;
	std::cout << "delta:\n" << y-x << "\n";
	std::cout << "out->hyperbolic:	" << out->hyperbolic-M_PI/2 << "\nout->spherical\n" << out->spherical << "\n";
	std::cout << "vector:\n" << vector << "\n";*/
	return Manifold::GeodesicPtr(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic(castedStart, end, vector, geodesic2d));
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::Geodesic(SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReferencePtr start, SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr end, Vector3d vector, std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> geodesic2d) {
	//assert(end->getSpherical() == end->getSpherical());
	this->start = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference>(start);
	this->end = std::tr1::static_pointer_cast<SurfaceOfRevolution<SurfaceOfRevolution2d>::Point>(end);
	this->vector = vector;
	this->geodesic2d = geodesic2d;
}

template <class SurfaceOfRevolution2d>
Manifold::PointPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::getEndPoint() {
	return std::tr1::static_pointer_cast<Manifold::Point>(end);
}

template <class SurfaceOfRevolution2d>
Vector3d SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::getVector() {
	return vector;
}

template <class SurfaceOfRevolution2d>
std::tr1::shared_ptr<typename SurfaceOfRevolution2d::Geodesic> SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::getGeodesic2d() {
	return geodesic2d;
}

template <class SurfaceOfRevolution2d>
void SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference::rotate(Matrix3d rot) {
	orientation.block<3,3>(0,0) = rot*orientation.block<3,3>(0,0);
}

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::intersectionDistance(Manifold::PortalPtr portal) {
	return intersectionDistance2(portal.get());
}

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::intersectionDistance2(Manifold::Portal* portal) {
	double dist = geodesic2d->intersectionDistance(((SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal*)portal)->getT());
	assert(dist > 0);
	return dist;
}

template <class SurfaceOfRevolution2d>
Manifold::PointOfReferencePtr SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic::getStart() {
	return std::tr1::static_pointer_cast<Manifold::PointOfReference>(start);
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal::Portal(bool side, Manifold* space) {
	setSpace(space);
	t = 2*M_PI/(log(2*M_PI)*getK());
	if(!side) {
		t = -t;
	}
}

template <class SurfaceOfRevolution2d>
SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal::Portal(double t, Manifold* space) {
	setSpace(space);
	this->t = t;
}

template <class SurfaceOfRevolution2d>
double SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal::getT() {
	return t;
}

template <class SurfaceOfRevolution2d>
Manifold::GeodesicPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal::getGeodesic(IntersectionPtr intersection) {
	double x = intersection->getPosition()[0];
	double y = intersection->getPosition()[1];
	double z = intersection->getPosition()[2];
	SurfaceOfRevolution<SurfaceOfRevolution2d>* space = (SurfaceOfRevolution<SurfaceOfRevolution2d>*) getSpace();
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointPtr startPoint(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(t,x,y,z,space));
	//std::cout << "Position of intersection after:\t" << startPoint->getT() << std::endl;
	/*SurfaceOfRevolution<typename SurfaceOfRevolution2d>::Point* startPointPointer = new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(t,x,y,z,space);
	SurfaceOfRevolution<typename SurfaceOfRevolution2d>::PointPtr startPoint(startPointPointer);*/
	//SurfaceOfRevolution<typename SurfaceOfRevolution2d>::PointPtr startPoint(new SurfaceOfRevolution<SurfaceOfRevolution2d>::Point(t,intersection->getPosition()[0],intersection->getPosition()[1],intersection->getPosition()[2],(SurfaceOfRevolution<SurfaceOfRevolution2d>*)getSpace()));
	/*MatrixXd<4,3> pre;
	Vector3d v = intersection->getPosition();
	pre << v,	v[0],0,0,	0,v[1],0,	0,0,v[2];*/
	
	Matrix4d orientation = Matrix4d::Identity();
	orientation.block<3,3>(0,0) = intersection->getOrientation();
	Vector4d p;
	p << intersection->getPosition(), -1;
	Matrix4d reflection = Matrix4d::Identity() - p*p.transpose();
	assert((reflection*reflection - Matrix4d::Identity()).squaredNorm() < EPSILON);
	Matrix4d s;
	s <<	0,0,0,1,
			1,0,0,0,
			0,1,0,0,
			0,0,1,0;
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReferencePtr start(new SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference(startPoint,s*reflection*orientation));
	return getSpace()->getGeodesic(start,intersection->getVector());
}

template <class SurfaceOfRevolution2d>
IntersectionPtr SurfaceOfRevolution<SurfaceOfRevolution2d>::Portal::getIntersection(Manifold::GeodesicPtr geodesic) {
	SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic* castedGeodesic = (SurfaceOfRevolution<SurfaceOfRevolution2d>::Geodesic*) geodesic.get();
	//Intersection2d intersection2d = castedGeodesic->getGeodesic2d()->getIntersection(getT());	//There's almost certainly a faster way using this, but it's not the time to optimize.
	double distance = castedGeodesic->intersectionDistance2(this);
	Vector3d vector = geodesic->getVector().normalized()*distance;
	assert(getSpace() != NULL);
	Manifold::PointOfReferencePtr end = getSpace()->getGeodesic(castedGeodesic->getStart(),vector)->getEndPointOfReference();
	//There is room for optimization here. I just recalculated a new geodesic, but most of the math is the same as the old one.
	assert(fabs(((SurfaceOfRevolution<SurfaceOfRevolution2d>::Point*)end->getPosition().get())->getT() - t) < EPSILON);
	SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference* castedEnd = (SurfaceOfRevolution<SurfaceOfRevolution2d>::PointOfReference*) end.get();
	//SurfaceOfRevolution<SurfaceOfRevolution2d>* castedEnd = (SurfaceOfRevolution<SurfaceOfRevolution2d>*) end.get();
	Matrix4d orientation = castedEnd->getOrientation();
	Vector3d position = ((SurfaceOfRevolution<SurfaceOfRevolution2d>::Point*)end->getPosition().get())->getSpherical();
	assert((orientation.transpose()*(Vector4d() << 0, position).finished()).start(3).norm() < EPSILON);
	/*Matrix<double,3,4> pre;
	pre.block<3,1>(0,0) = position;
	pre.block<3,3>(0,1) = Matrix3d::Identity();
	Matrix3d final = pre*orientation.block<4,3>(0,0);
	std::cout << "orientation:\n" << final << std::endl;
	assert(fabs(final.determinant()-1) < EPSILON);
	#ifndef NDEBUG
	Matrix4d temp = -Matrix4d::Identity();
	temp.block<3,3>(0,0) = final;
	Vector4d p;
	p << position, -1;
	Matrix4d reflection = Matrix4d::Identity() - p*p.transpose();
	assert((reflection*temp - orientation).squaredNorm() < EPSILON);
	#endif*/
	Vector4d p;
	p << position, -1;
	Matrix4d reflection = Matrix4d::Identity() - p*p.transpose();
	Matrix4d ss;
	ss <<	0,1,0,0,
			0,0,1,0,
			0,0,0,1,
			1,0,0,0;
	Matrix4d final = reflection*ss*orientation;
	/*std::cout << "final:\n" << final << std::endl;
	Vector4d v;
	v << 0,position;
	std::cout << "orientation*(0,0,0,1):\n" << orientation*Vector4d(0,0,0,1) << std::endl;
	std::cout << "orientation*reflection*(0,v):\n" << orientation*reflection*v << std::endl;
	std::cout << "final*(0,0,0,1):\n" << final*Vector4d(0,0,0,1) << std::endl;*/
	assert((final.block<1,4>(3,0).transpose() - Vector4d(0,0,0,1)).squaredNorm() < EPSILON);
	assert((final.block<4,1>(0,3) - Vector4d(0,0,0,1)).squaredNorm() < EPSILON);
	/*std::cout << "Original length:\t" << geodesic->getVector().norm() << std::endl;
	std::cout << "Distance to portal:\t" << distance << std::endl;
	std::cout << "Remaining distance:\t" << (geodesic->getVector()-vector).norm() << std::endl;*/
	return IntersectionPtr(new Intersection(position, final.block<3,3>(0,0), geodesic->getVector()-vector));
}

#endif

