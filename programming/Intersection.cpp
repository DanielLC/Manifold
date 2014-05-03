#include "Intersection.h"
#include "Assert.h"

Vector3d Intersection::getPosition() {
	return position;
}

Matrix3d Intersection::getOrientation() {
	return orientation;
}

Vector3d Intersection::getVector() {
	return vector;
}

Intersection::Intersection(Vector3d position, Matrix3d orientation, Vector3d vector) {
	this->position = position;
	assert(fabs(position.squaredNorm()-1) < EPSILON);
	this->orientation = orientation;
	this->vector = vector;
}

void Intersection::rotate(Matrix3d rotation) {
	position = rotation*position;
	orientation = rotation*orientation;
}

void Intersection::invert() {
	Matrix3d reflection;
	reflection <<	Vector3d(1,0,0) - 2*position[0]*position,
					Vector3d(0,1,0) - 2*position[1]*position,
					Vector3d(0,0,1) - 2*position[2]*position;
	orientation = reflection*orientation;
}

