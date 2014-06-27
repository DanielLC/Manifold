#ifndef image_h
#define image_h

#include <tr1/array>
#include "Color.h"
#include "Constants.h"

typedef std::tr1::array<std::tr1::array<Color,WIDTH>,HEIGHT> Image;
typedef std::tr1::shared_ptr<Image> ImagePtr;

#endif
