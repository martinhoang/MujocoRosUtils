#include "depth_processing_cuda.cuh"

#include <cuda_runtime.h>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace cuda_img_proc {

// ---------------------------------------------------------------------------
// Block size: 16×16 = 256 threads per block — fits any modern GPU warp size.
// ---------------------------------------------------------------------------
static constexpr int BLOCK_X = 16;
static constexpr int BLOCK_Y = 16;

// ===========================================================================
// Kernel 1: Depth linearisation + vertical flip
// ===========================================================================
__global__ void kernel_depth_linearize_flip(
    const float* __restrict__ raw,    // H×W, OpenGL depth, bottom-row-first
    float* __restrict__       lin,    // H×W, metres, top-row-first (output)
    int W, int H,
    float near, float depth_scale,    // depth_scale = 1 - near/far
    int zerofar_conv)                 // 1 = data is 0=far (needs 1-x flip)
{
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  if (u >= W || v >= H) return;

  // Flip vertically: OpenGL bottom-origin → top-origin
  float d = raw[(H - 1 - v) * W + u];

  // Normalize to ZERONEAR (0=near, 1=far) if data is in ZEROFAR convention
  if (zerofar_conv) d = 1.0f - d;

  // Linearise: depth_m = near / (1 - d * (1 - near/far))
  lin[v * W + u] = near / (1.0f - d * depth_scale);
}

void launchDepthLinearizeFlip(
    const float* d_raw_depth,
    float* d_lin_depth,
    int W, int H,
    float near, float far,
    int zerofar_conv,
    cudaStream_t stream)
{
  const float depth_scale = 1.0f - near / far;
  const dim3  block(BLOCK_X, BLOCK_Y);
  const dim3  grid((W + BLOCK_X - 1) / BLOCK_X, (H + BLOCK_Y - 1) / BLOCK_Y);
  kernel_depth_linearize_flip<<<grid, block, 0, stream>>>(
      d_raw_depth, d_lin_depth, W, H, near, depth_scale, zerofar_conv);
}

// ===========================================================================
// Kernel 2: Depth → PointCloud2
// ===========================================================================
//
// Rotation preset IDs — must match rotationPresetToInt() below and the
// PCL_ROT_PRESET_* strings in depth_conversions.hpp.
//
// 0  none
// 1  ROLL_180
// 2  RDF_TO_FLU
// 3  ROLL_90
// 4  N_ROLL_90
// 5  PITCH_90
// 6  YAW_90

__global__ void kernel_depth_to_cloud(
    const float* __restrict__   lin_depth,   // H×W, metres, top-row-first
    const uint8_t* __restrict__ rgb_src,     // H×W×3, RGB
    int   color_y_flip,                      // 1 = rgb_src is bottom-row-first
    uint8_t* __restrict__       cloud_out,   // H×W × point_step (output)
    int W, int H,
    float fx, float fy, float cx, float cy,
    float range_min, float range_max,
    uint32_t point_step,
    uint32_t off_x, uint32_t off_y, uint32_t off_z, uint32_t off_rgb,
    int   rotation_preset,
    float bad_point)
{
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  if (u >= W || v >= H) return;

  // Pointers into the output cloud buffer for this pixel
  uint8_t* p    = cloud_out + (static_cast<size_t>(v) * W + u) * point_step;
  float*   px   = reinterpret_cast<float*>(p + off_x);
  float*   py   = reinterpret_cast<float*>(p + off_y);
  float*   pz   = reinterpret_cast<float*>(p + off_z);
  int*     prgb = reinterpret_cast<int*>(p + off_rgb);

  const float depth = lin_depth[v * W + u];

  // Validity check
  const bool depth_ok = isfinite(depth) && depth > 0.0f
                        && (range_max <= 0.0f || depth <= range_max)
                        && (range_min <= 0.0f || depth >= range_min);

  if (!depth_ok) {
    *px = *py = *pz = bad_point;
    *prgb = 0;
    return;
  }

  // Unproject
  float x = (static_cast<float>(u) - cx) * depth / fx;
  float y = (static_cast<float>(v) - cy) * depth / fy;
  float z = depth;

  // Apply rotation preset
  const float tx = x, ty = y, tz = z;
  switch (rotation_preset) {
    case 1: x = -tx; y = -ty;                break;  // ROLL_180
    case 2: x =  tz; y = -tx; z = -ty;      break;  // RDF_TO_FLU
    case 3: x =  tx; y = -tz; z =  ty;      break;  // ROLL_90
    case 4: x = -tx; y =  tz; z = -ty;      break;  // N_ROLL_90
    case 5: x =  tz; y =  ty; z = -tx;      break;  // PITCH_90
    case 6: x = -ty; y =  tx;               break;  // YAW_90
    default: break;
  }

  *px = x;
  *py = y;
  *pz = z;

  // Pack RGB into a 32-bit int (R in high byte, matching PCL convention)
  const int color_row   = color_y_flip ? (H - 1 - v) : v;
  const uint8_t* rgb_px = rgb_src + (static_cast<size_t>(color_row) * W + u) * 3;
  const int packed = (static_cast<int>(rgb_px[0]) << 16)
                   | (static_cast<int>(rgb_px[1]) <<  8)
                   |  static_cast<int>(rgb_px[2]);
  memcpy(prgb, &packed, sizeof(int));
}

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
    cudaStream_t stream)
{
  const dim3 block(BLOCK_X, BLOCK_Y);
  const dim3 grid((W + BLOCK_X - 1) / BLOCK_X, (H + BLOCK_Y - 1) / BLOCK_Y);
  kernel_depth_to_cloud<<<grid, block, 0, stream>>>(
      d_lin_depth, d_color_rgb, color_y_flip,
      d_cloud_out, W, H,
      fx, fy, cx, cy,
      range_min, range_max,
      point_step, off_x, off_y, off_z, off_rgb,
      rotation_preset, bad_point);
}

// ===========================================================================
// Rotation preset name → integer
// ===========================================================================
int rotationPresetToInt(const std::string& preset_name)
{
  if (preset_name == "ROLL_180")  return 1;
  if (preset_name == "RDF_TO_FLU") return 2;
  if (preset_name == "ROLL_90")   return 3;
  if (preset_name == "N_ROLL_90") return 4;
  if (preset_name == "PITCH_90")  return 5;
  if (preset_name == "YAW_90")    return 6;
  return 0;  // none / unknown
}

// ===========================================================================
// CudaImageBuffers — GPU + pinned host memory management
// ===========================================================================
void CudaImageBuffers::allocate(int W, int H, int ps)
{
  release();

  width      = W;
  height     = H;
  point_step = ps;

  const size_t n_px    = static_cast<size_t>(W) * H;
  const size_t n_rgb   = n_px * 3;
  const size_t n_cloud = n_px * static_cast<size_t>(ps);

  cudaMalloc(reinterpret_cast<void**>(&d_depth_raw),    n_px    * sizeof(float));
  cudaMalloc(reinterpret_cast<void**>(&d_depth_linear), n_px    * sizeof(float));
  cudaMalloc(reinterpret_cast<void**>(&d_color_raw),    n_rgb);
  cudaMalloc(reinterpret_cast<void**>(&d_cloud),        n_cloud);

  cudaMallocHost(reinterpret_cast<void**>(&h_depth_linear), n_px * sizeof(float));
  cudaMallocHost(reinterpret_cast<void**>(&h_cloud),        n_cloud);
}

void CudaImageBuffers::release()
{
  if (d_depth_raw)    { cudaFree(d_depth_raw);    d_depth_raw    = nullptr; }
  if (d_depth_linear) { cudaFree(d_depth_linear); d_depth_linear = nullptr; }
  if (d_color_raw)    { cudaFree(d_color_raw);    d_color_raw    = nullptr; }
  if (d_cloud)        { cudaFree(d_cloud);        d_cloud        = nullptr; }
  if (h_depth_linear) { cudaFreeHost(h_depth_linear); h_depth_linear = nullptr; }
  if (h_cloud)        { cudaFreeHost(h_cloud);        h_cloud        = nullptr; }

  width = height = point_step = 0;
}

} // namespace cuda_img_proc
