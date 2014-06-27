#ifndef image_out_h
#define image_out_h

#include <string>
#include "Image.h"

class ImageOut {
	public:
		static void draw(ImagePtr image, std::string file);
		static void draw();
};

#endif
