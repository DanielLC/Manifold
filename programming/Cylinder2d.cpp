#include <tr1/array>
#include <tr1/memory>
#include <iostream>

#include "Cylinder2d.h"
#include <Eigen/Dense>
#include <utility>
using Eigen::Vector2d;

Cylinder2d::Point::Point() {
}

Cylinder2d::Point::Point(double t, double thetak) {
	coordinates[0] = t;
	coordinates[1] = thetak;
}

double Cylinder2d::Point::getT(){
	return coordinates[0];
}

double Cylinder2d::Point::getThetaK(){
	return coordinates[1];
}

std::tr1::array<double,2> Cylinder2d::Point::getCoordinates(){
	return coordinates;
}

Vector2d Cylinder2d::Point::vectorFromPoint(Cylinder2d::Point point) {
	return Vector2d(point.coordinates[0] - coordinates[0], point.coordinates[1] - coordinates[1]);
}

Cylinder2d::Point Cylinder2d::Point::pointFromVector(Vector2d z) {
	return Cylinder2d::Point(coordinates[0] + z[0], coordinates[1] + z[1]);
}

void Cylinder2d::Point::setCoordinates(double t, double thetak) {
	coordinates[0] = t;
	coordinates[1] = thetak;
}

std::pair<Cylinder2d::Point, double> Cylinder2d::Point::pointAndRotFromVector(Vector2d z) {
	return std::pair<Cylinder2d::Point, double>(pointFromVector(z), 0.);
}

