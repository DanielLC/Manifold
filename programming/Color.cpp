#include "Color.h"

Color::Color() {
	this->red = 0;
	this->green = 0;
	this->blue = 0;
}

Color::Color(double red, double green, double blue) {
	this->red = red;
	this->green = green;
	this->blue = blue;
}

/*Color::Color(char red, char green, char blue) {
	this->red = red/255.0;
	this->green = green/255.0;
	this->blue = blue/255.0;
}*/

double Color::getRed() {
	return red;
}

double Color::getGreen() {
	return green;
}

double Color::getBlue() {
	return blue;
}

double Color::total() {
	return red+green+blue;
}

Color Color::operator+(Color color) {
	double newRed = red + color.getRed();
	double newGreen = green + color.getGreen();
	double newBlue = blue + color.getBlue();
	return Color(newRed, newGreen, newBlue);
}

void Color::operator+=(Color color) {
	red += color.getRed();
	green += color.getGreen();
	blue += color.getBlue();
}

Color Color::operator*(Color color) {
	double newRed = red*color.getRed();
	double newGreen = green*color.getGreen();
	double newBlue = blue*color.getBlue();
	return Color(newRed, newGreen, newBlue);
}

void Color::operator*=(Color color) {
	red *= color.getRed();
	green *= color.getGreen();
	blue *= color.getBlue();
}

Color Color::operator*(double k) {
	double newRed = k*red;
	double newGreen = k*green;
	double newBlue = k*blue;
	return Color(newRed, newGreen, newBlue);
}

void Color::operator*=(double k) {
	red *= k;
	green *= k;
	blue *= k;
}

Color Color::operator/(double k) {
	double newRed = red/k;
	double newGreen = green/k;
	double newBlue = blue/k;
	return Color(newRed, newGreen, newBlue);
}

void Color::operator/=(double k) {
	red /= k;
	green /= k;
	blue /= k;
}

