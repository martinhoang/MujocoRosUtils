#include "lodepng.h"

#include <png.h>

namespace lodepng {

unsigned encode(
  const std::string & filename,
  const unsigned char * image,
  unsigned width,
  unsigned height,
  LodePNGColorType color_type,
  unsigned bit_depth)
{
  if (!image || width == 0 || height == 0 || bit_depth != 8) {
    return 1;
  }

  png_image png = {};
  png.version = PNG_IMAGE_VERSION;
  png.width = width;
  png.height = height;

  switch (color_type) {
    case LCT_GREY:
      png.format = PNG_FORMAT_GRAY;
      break;
    case LCT_GREY_ALPHA:
      png.format = PNG_FORMAT_GA;
      break;
    case LCT_RGB:
      png.format = PNG_FORMAT_RGB;
      break;
    case LCT_RGBA:
      png.format = PNG_FORMAT_RGBA;
      break;
    default:
      return 1;
  }

  return png_image_write_to_file(
           &png, filename.c_str(), 0, image, 0, nullptr)
           ? 0
           : 1;
}

}  // namespace lodepng
