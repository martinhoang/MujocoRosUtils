// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros-perception/image_pipeline/blob/da750d1/depth_image_proc/include/depth_image_proc/depth_conversions.h
// // NOLINT

#ifndef DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_CONVERSIONS_HPP_
#define DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_CONVERSIONS_HPP_

#include "depth_traits.hpp"

#include <rclcpp/version.h>
#if RCLCPP_VERSION_MAJOR >= 28
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#endif

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <opencv2/imgproc/imgproc.hpp>

namespace depthimage_to_pointcloud2
{

constexpr char PCL_ROT_NO_PRESET[]         = "";           // No rotation
constexpr char PCL_ROT_PRESET_ROLL_180[]   = "roll_180";   // Default MuJoCo to ROS
constexpr char PCL_ROT_PRESET_RDF_TO_FLU[] = "rdf_to_flu"; // Standard ROS camera to standard robot
constexpr char PCL_ROT_PRESET_ROLL_90[]    = "roll_90";
constexpr char PCL_ROT_PRESET_N_ROLL_90[]  = "roll_n90";
constexpr char PCL_ROT_PRESET_PITCH_90[]   = "pitch_90";
constexpr char PCL_ROT_PRESET_YAW_90[]     = "yaw_90";

// Handles float or uint16 depths
// num_threads: number of OpenMP threads to use for the pixel loop.
//   Pass 1 (default) for single-threaded. When >1, requires OpenMP.
template <typename T>
void convert(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
             sensor_msgs::msg::PointCloud2::SharedPtr      &cloud_msg,
             const image_geometry::PinholeCameraModel &model, double range_max = 0.0,
             bool use_quiet_nan = false, const std::string &rotation_preset = "",
             cv_bridge::CvImageConstPtr cv_ptr = nullptr, double range_min = 0.0,
             int num_threads = 1)
{
  // Use correct principal point from calibration
  const float center_x   = model.cx();
  const float center_y   = model.cy();
  const float constant_x = static_cast<float>(DepthTraits<T>::toMeters(T(1)) / model.fx());
  const float constant_y = static_cast<float>(DepthTraits<T>::toMeters(T(1)) / model.fy());
  const float bad_point  = std::numeric_limits<float>::quiet_NaN();

  // Pre-compute range limits as depth-type values once (avoids per-pixel conversion)
  const T depth_max_t = (range_max != 0.0) ? DepthTraits<T>::fromMeters(range_max) : T(0);
  const T depth_min_t = (range_min != 0.0) ? DepthTraits<T>::fromMeters(range_min) : T(0);

  // Resolve byte offsets for x, y, z, rgb fields in the cloud point_step layout.
  // Using raw pointer arithmetic instead of PointCloud2Iterator enables true random
  // access into any row, making the outer loop safely parallelisable with OpenMP.
  uint32_t off_x = 0, off_y = 4, off_z = 8, off_rgb = 12;
  for (const auto & f : cloud_msg->fields)
  {
    if (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
    else if (f.name == "rgb") off_rgb = f.offset;
  }

  uint8_t *      cloud_data  = cloud_msg->data.data();
  const uint32_t point_step  = cloud_msg->point_step;
  const T *      depth_data  = reinterpret_cast<const T *>(depth_msg->data.data());
  const int      row_step    = static_cast<int>(depth_msg->step / sizeof(T));
  const int      height      = static_cast<int>(cloud_msg->height);
  const int      width       = static_cast<int>(cloud_msg->width);
  const bool     has_rgb     = (cv_ptr != nullptr);
  const bool     is_gray     = has_rgb && (cv_ptr->image.type() == CV_8UC1);
  const bool     is_bgr      = has_rgb && (cv_ptr->image.type() == CV_8UC3);

#ifdef _OPENMP
  // Cap threads to what OpenMP has available
  const int n_threads = std::min(num_threads, omp_get_max_threads());
#pragma omp parallel for schedule(static) num_threads(n_threads)
#endif
  for (int v = 0; v < height; ++v)
  {
    const T * depth_row_ptr = depth_data + v * row_step;

    for (int u = 0; u < width; ++u)
    {
      // Direct pointer into flat cloud buffer — no iterator copies needed
      uint8_t * p    = cloud_data + static_cast<size_t>(v * width + u) * point_step;
      float *   px   = reinterpret_cast<float *>(p + off_x);
      float *   py   = reinterpret_cast<float *>(p + off_y);
      float *   pz   = reinterpret_cast<float *>(p + off_z);
      int *     prgb = reinterpret_cast<int *>(p + off_rgb);

      T depth = depth_row_ptr[u];

      // Missing / out-of-range points → NaN
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0 && !use_quiet_nan)
          depth = depth_max_t;
        else
        {
          *px = *py = *pz = bad_point;
          *prgb            = 0;
          continue;
        }
      }

      if (range_max != 0.0 && depth > depth_max_t)
      {
        *px = *py = *pz = bad_point;
        *prgb            = 0;
        continue;
      }

      if (range_min != 0.0 && depth < depth_min_t)
      {
        *px = *py = *pz = bad_point;
        *prgb            = 0;
        continue;
      }

      // Unproject depth pixel to 3-D point
      float x = (u - center_x) * depth * constant_x;
      float y = (v - center_y) * depth * constant_y;
      float z = DepthTraits<T>::toMeters(depth);

      if (!rotation_preset.empty())
      {
        const float tx = x, ty = y, tz = z;
        if (rotation_preset == PCL_ROT_PRESET_ROLL_180)
        {
          x = -tx;
          y = -ty;
        }
        else if (rotation_preset == PCL_ROT_PRESET_RDF_TO_FLU)
        {
          x = tz;
          y = -tx;
          z = -ty;
        }
        else if (rotation_preset == PCL_ROT_PRESET_ROLL_90)
        {
          x = tx;
          y = -tz;
          z = ty;
        }
        else if (rotation_preset == PCL_ROT_PRESET_N_ROLL_90)
        {
          x = -tx;
          y = tz;
          z = -ty;
        }
        else if (rotation_preset == PCL_ROT_PRESET_PITCH_90)
        {
          x = tz;
          y = ty;
          z = -tx;
        }
        else if (rotation_preset == PCL_ROT_PRESET_YAW_90)
        {
          x = -ty;
          y = tx;
          z = tz;
        }
      }

      *px = x;
      *py = y;
      *pz = z;

      // RGB — cv::Mat::at<> is safe for concurrent reads from different (v,u) pairs
      int rgb = 0;
      if (has_rgb)
      {
        if (is_gray)
        {
          const int g = cv_ptr->image.at<uchar>(v, u);
          rgb         = (g << 16) | (g << 8) | g;
        }
        else if (is_bgr)
        {
          const cv::Vec3b & px_bgr = cv_ptr->image.at<cv::Vec3b>(v, u);
          rgb                      = (px_bgr[0] << 16) | (px_bgr[1] << 8) | px_bgr[2];
        }
      }
      std::memcpy(prgb, &rgb, sizeof(int));
    }
  }
}

} // namespace depthimage_to_pointcloud2

#endif // DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_CONVERSIONS_HPP_