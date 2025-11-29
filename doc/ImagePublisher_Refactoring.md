# ImagePublisher Plugin Refactoring Recommendations

## Current Issues
- Single 1500+ line file with mixed concerns (OpenGL, ROS, threading, MuJoCo)
- Long constructor with 14 parameters
- Duplicated image processing logic
- Hard to test and maintain

## Proposed Modular Structure

```
plugin/
├── ImagePublisher.cpp           # Main orchestrator (simplified)
├── ImagePublisher.h
├── image_publisher/
│   ├── GLContext.{cpp,h}        # OpenGL window & rendering
│   ├── ImageBuffer.{cpp,h}      # Memory management & conversions
│   ├── PBOReader.{cpp,h}        # Async GPU readback
│   ├── ROSPublisher.{cpp,h}     # ROS2 topic publishing
│   ├── ImageUtils.{cpp,h}       # Flip, BGR conversion, parallel ops
│   └── Config.{cpp,h}           # Configuration parsing
```

## Component Responsibilities

### GLContext
- GLFW window creation & management
- MuJoCo render context (mjrContext, mjvScene)
- Offscreen rendering
- **~150 lines**

### ImageBuffer
- Color/depth buffer allocation
- Double buffering for thread safety
- Format conversions
- **~100 lines**

### PBOReader
- OpenGL Pixel Buffer Objects
- Async readback with fences
- PBO cycling logic
- **~120 lines**

### ROSPublisher
- ROS2 node & publishers
- CameraInfo, Image, PointCloud2 publishing
- Subscriber count checking
- **~150 lines**

### ImageUtils
- `flipAndConvertRGBtoBGR()` - with/without parallelization
- `convertDepthBuffer()` - depth linearization
- Reusable static methods
- **~80 lines**

### Config
- Parse MuJoCo XML attributes
- Validation & defaults
- Replaces 14-parameter constructor
- **~100 lines**

### ImagePublisher (Main)
- Plugin registration
- Component composition & coordination
- Compute loop: render → readback → publish
- **~200 lines** (down from 1500+)

## Benefits

1. **Testability**: Unit test each component independently
2. **Maintainability**: Changes localized to single file
3. **Compile Time**: Edit GL code without recompiling ROS
4. **Readability**: Clear separation of concerns
5. **Reusability**: PBOReader/ImageUtils usable elsewhere

## Thread Safety Improvements

Current: Mutex + boolean flags
Proposed: Producer-consumer queue pattern

```cpp
template<typename T>
class FrameQueue {
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
public:
    void push(T&& frame);
    bool try_pop(T& frame, std::chrono::milliseconds timeout);
};
```

## Configuration Simplification

**Before:**
```cpp
ImagePublisher(m, d, sensor_id, frame_id, topic_namespace, 
               color_topic, depth_topic, info_topic, pointcloud_topic,
               rotate_pcl, rotation_preset, height, width, 
               publish_rate, max_range, readback_mode, enable_parallel);
```

**After:**
```cpp
auto config = ImagePublisherConfig::fromMujocoPlugin(m, plugin_id);
ImagePublisher(m, config);
```

## Implementation Priority

1. Extract `Config` - immediate benefit to readability
2. Extract `ImageUtils` - removes duplication
3. Extract `GLContext` - isolates OpenGL complexity
4. Extract `ROSPublisher` - separates ROS concerns
5. Extract `PBOReader` and `ImageBuffer` - completes refactor

## Notes

- Maintain backward compatibility with existing XML configs
- Keep plugin registration interface unchanged
- Use `std::unique_ptr` for component ownership
- Consider adding `ImagePublisher::Builder` for complex initialization
