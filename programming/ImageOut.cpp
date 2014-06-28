#include "ImageOut.h"
#include "Constants.h"
#include <png++/png.hpp>

void ImageOut::draw(ImagePtr image, std::string file) {
	png::image< png::rgb_pixel > image2(WIDTH, HEIGHT);
	for (size_t y = 0; y < image2.get_height(); ++y) {
		for (size_t x = 0; x < image2.get_width(); ++x) {
			int red = std::min(255,(int)((*image)[y][x].getRed()*255));
			int green = std::min(255,(int)((*image)[y][x].getGreen()*255));
			int blue = std::min(255,(int)((*image)[y][x].getBlue()*255));
			/*if(red > 255 || red < 0 || green > 255 || green < 0 || blue > 255 || blue < 0) {
				std::cout << "Error: color out of bounds." << std::endl;
			}*/
			image2[y][x] = png::rgb_pixel(red, green, blue);
			/*Color color = (*image)[y][x];
			image2[y][x] = png::rgb_pixel(color.getRed(), color.getGreen(), color.getBlue());*/
			//image2[y][x] = png::rgb_pixel(0, 0, 0);
		}
	}
	image2.write(file);
}
