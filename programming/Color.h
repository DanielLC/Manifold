#ifndef color_h
#define color_h

#include <tr1/memory>

class Color {
	public:
		Color();
		Color(double red, double green, double blue);
		//Color(char red, char green, char blue);
		double getRed();
		double getGreen();
		double getBlue();
		double total();
		Color operator+(Color color);
		void operator+=(Color color);
		Color operator*(Color color);
		void operator*=(Color color);
		Color operator*(double k);
		void operator*=(double k);
		Color operator/(double k);
		void operator/=(double k);
	private:
		double red;
		double green;
		double blue;
};

//typedef std::tr1::shared_ptr<Color> ColorPtr;

#endif
