#ifndef MUJOCO_ROS_UTILS_LODEPNG_H_
#define MUJOCO_ROS_UTILS_LODEPNG_H_

#include <string>

enum LodePNGColorType {
  LCT_GREY = 0,
  LCT_RGB = 2,
  LCT_PALETTE = 3,
  LCT_GREY_ALPHA = 4,
  LCT_RGBA = 6,
};

namespace lodepng {

unsigned encode(
  const std::string & filename,
  const unsigned char * image,
  unsigned width,
  unsigned height,
  LodePNGColorType color_type = LCT_RGBA,
  unsigned bit_depth = 8);

}  // namespace lodepng

#endif  // MUJOCO_ROS_UTILS_LODEPNG_H_
