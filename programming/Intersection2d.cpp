#include "Intersection2d.h"
#include <Eigen/Core>
using Eigen::Vector2d;

Intersection2d::Intersection2d(double position, double rotation, Vector2d vector) {
	this->position = position;
	this->rotation = rotation;
	this->vector = vector;
}

double Intersection2d::getPosition() {
	return position;
}

double Intersection2d::getRotation() {
	return rotation;
}

Vector2d Intersection2d::getVector(){
	return vector;
}

