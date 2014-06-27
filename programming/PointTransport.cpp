#include "PointTransport.h"
#include "Assert.h"

Vector3d PointTransport::getPosition() {
	return position;
}

double PointTransport::getDistance() {
	return distance;
}

PointTransport::PointTransport(Vector3d position, double distance) {
	assert(fabs(position.squaredNorm()-1) < EPSILON);
	this->position = position;
	//std::cout << "PointTransport.cpp distance:\t" << distance << std::endl;
	assert(distance > -0.0001);
	this->distance = distance;
}

