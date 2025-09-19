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

#include <limits>
#include <string>

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
template <typename T>
void convert(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
             sensor_msgs::msg::PointCloud2::SharedPtr      &cloud_msg,
             const image_geometry::PinholeCameraModel &model, double range_max = 0.0,
             bool use_quiet_nan = false, const std::string &rotation_preset = "",
             cv_bridge::CvImageConstPtr cv_ptr = nullptr)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  float  constant_x   = unit_scaling / model.fx();
  float  constant_y   = unit_scaling / model.fy();
  float  bad_point    = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(*cloud_msg, "rgb");
  const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int      row_step  = depth_msg->step / sizeof(T);
#pragma omp parallel for
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v)
  {
    const T                                *depth_row_ptr = depth_row + v * row_step;
    sensor_msgs::PointCloud2Iterator<float> iter_x_local(iter_x + v * cloud_msg->width);
    sensor_msgs::PointCloud2Iterator<float> iter_y_local(iter_y + v * cloud_msg->width);
    sensor_msgs::PointCloud2Iterator<float> iter_z_local(iter_z + v * cloud_msg->width);
    sensor_msgs::PointCloud2Iterator<float> iter_rgb_local(iter_rgb + v * cloud_msg->width);

    for (int u = 0; u < static_cast<int>(cloud_msg->width);
         ++u, ++iter_x_local, ++iter_y_local, ++iter_z_local, ++iter_rgb_local)
    {
      T depth = depth_row_ptr[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0 && !use_quiet_nan)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x_local = *iter_y_local = *iter_z_local = *iter_rgb_local = bad_point;
          continue;
        }
      }

      if (range_max != 0.0)
      {
        T depth_max = DepthTraits<T>::fromMeters(range_max);
        if (depth > depth_max)
        {
          *iter_x_local = *iter_y_local = *iter_z_local = *iter_rgb_local = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      float x = (u - center_x) * depth * constant_x;
      float y = (v - center_y) * depth * constant_y;
      float z = DepthTraits<T>::toMeters(depth);

      if (!rotation_preset.empty())
      {
        float temp_x = x, temp_y = y, temp_z = z;
        if (rotation_preset == PCL_ROT_NO_PRESET)
        {
          // do nothing
        }
        else if (rotation_preset == PCL_ROT_PRESET_ROLL_180)
        { // Default MuJoCo to ROS
          x = -temp_x;
          y = -temp_y;
        }
        else if (rotation_preset == PCL_ROT_PRESET_RDF_TO_FLU)
        { // Standard ROS camera to standard robot
          x = temp_z;
          y = -temp_x;
          z = -temp_y;
        }
        else if (rotation_preset == PCL_ROT_PRESET_ROLL_90)
        {
          x = temp_x;
          y = -temp_z;
          z = temp_y;
        }
        else if (rotation_preset == PCL_ROT_PRESET_N_ROLL_90)
        {
          x = -temp_x;
          y = temp_z;
          z = -temp_y;
        }
        else if (rotation_preset == PCL_ROT_PRESET_PITCH_90)
        {
          x = temp_z;
          y = temp_y;
          z = -temp_x;
        }
        else if (rotation_preset == PCL_ROT_PRESET_YAW_90)
        {
          x = -temp_y;
          y = temp_x;
          z = temp_z;
        }
      }
      *iter_x_local = x;
      *iter_y_local = y;
      *iter_z_local = z;

      // and RGB
      int rgb = 0x000000;
      if (cv_ptr != nullptr)
      {
        if (cv_ptr->image.type() == CV_8UC1)
        {
          // grayscale
          rgb = cv_ptr->image.at<uchar>(v, u);
          rgb = (rgb << 16) | (rgb << 8) | rgb; // Convert grayscale to RGB
        }
        else if (cv_ptr->image.type() == CV_8UC3)
        {
          // RGB
          cv::Vec3b intensity = cv_ptr->image.at<cv::Vec3b>(v, u);
          rgb                 = (intensity[0] << 16) | (intensity[1] << 8)
                | intensity[2]; // Convert BGR to RGB correctly
        }
      }
      std::memcpy(&(*iter_rgb_local), &rgb, sizeof(int));
    }
  }
}

} // namespace depthimage_to_pointcloud2

#endif // DEPTHIMAGE_TO_POINTCLOUD2__DEPTH_CONVERSIONS_HPP_