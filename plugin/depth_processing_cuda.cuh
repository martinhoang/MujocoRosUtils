#pragma once
#include <cuda_runtime.h>
#include <cstdint>
#include <string>

namespace cuda_img_proc {

/**
 * Linearize OpenGL depth buffer and flip vertically in one CUDA kernel.
 *
 * @param d_raw_depth  device ptr — H×W floats, OpenGL NDC depth, bottom-row-first
 * @param d_lin_depth  device ptr — H×W floats, linearized meters, top-row-first (output)
 * @param W, H         image dimensions
 * @param near, far    frustum clip planes (metres)
 * @param zerofar_conv 1 if data is in ZEROFAR convention (needs 1-x flip), 0 if already ZERONEAR
 * @param stream       CUDA stream
 */
void launchDepthLinearizeFlip(
    const float* d_raw_depth,
    float* d_lin_depth,
    int W, int H,
    float near, float far,
    int zerofar_conv,
    cudaStream_t stream);

/**
 * Back-project linearized depth + RGB into PointCloud2 data layout.
 *
 * @param d_lin_depth    device ptr — H×W floats, meters, top-row-first
 * @param d_color_rgb    device ptr — H×W×3 uint8, RGB
 * @param color_y_flip   1 if d_color_rgb is bottom-row-first (OpenGL readback), 0 if top-row-first
 * @param d_cloud_out    device ptr — H×W × point_step bytes (output)
 * @param W, H           image dimensions
 * @param fx, fy, cx, cy camera intrinsics
 * @param range_min/max  distance filter in metres (0 = disabled)
 * @param point_step     bytes per PointCloud2 point
 * @param off_x,y,z,rgb  byte offsets within point_step for each field
 * @param rotation_preset rotation preset index (see rotationPresetToInt())
 * @param bad_point      NaN value placed for invalid/out-of-range points
 * @param stream         CUDA stream
 */
void launchDepthToCloud(
    const float* d_lin_depth,
    const uint8_t* d_color_rgb,
    int color_y_flip,
    uint8_t* d_cloud_out,
    int W, int H,
    float fx, float fy, float cx, float cy,
    float range_min, float range_max,
    uint32_t point_step,
    uint32_t off_x, uint32_t off_y, uint32_t off_z, uint32_t off_rgb,
    int rotation_preset,
    float bad_point,
    cudaStream_t stream);

/** Convert rotation preset string (from depth_conversions.hpp) to integer for the CUDA kernel. */
int rotationPresetToInt(const std::string& preset_name);

/**
 * GPU + pinned-host buffer set for one image resolution.
 * Call allocate() before first use and whenever the resolution changes.
 * Call release() in the destructor (or on cleanup).
 */
struct CudaImageBuffers
{
  float*   d_depth_raw    = nullptr;  ///< H×W float — raw depth from PBO (device)
  float*   d_depth_linear = nullptr;  ///< H×W float — linearised, top-down (device)
  uint8_t* d_color_raw    = nullptr;  ///< H×W×3 uint8 — RGB from PBO (device)
  uint8_t* d_cloud        = nullptr;  ///< H×W×point_step uint8 — cloud data (device)
  float*   h_depth_linear = nullptr;  ///< H×W float — pinned host output
  uint8_t* h_cloud        = nullptr;  ///< H×W×point_step uint8 — pinned host output

  int width      = 0;
  int height     = 0;
  int point_step = 0;

  /** Allocate all buffers for the given image size and cloud point_step. */
  void allocate(int W, int H, int ps);

  /** Free all buffers (safe to call even if not allocated). */
  void release();

  bool isAllocated() const { return d_depth_raw != nullptr; }
};

} // namespace cuda_img_proc
