#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace MujocoRosUtils
{

/**
 * One rendered color frame produced by an ImagePublisher instance.
 *
 * Format: BGR, top-to-bottom, row-major (height × width × 3 bytes).
 * This matches the OpenCV Mat layout so callers can wrap it with
 *   cv::Mat img(height, width, CV_8UC3, frame.data.data());
 * without any copies.
 */
struct CameraFrame
{
  std::vector<uint8_t> data;   ///< BGR pixel data, height × width × 3 bytes
  int      height   = 0;
  int      width    = 0;
  double   sim_time = 0.0;     ///< mjData::time when frame was rendered
  bool     valid    = false;
  uint64_t seq      = 0;       ///< monotonically increases with each updateCameraFrame() call
};

/**
 * State + command snapshot for a single 1-DOF joint (hinge or slide).
 * Multi-DOF joints (free, ball) are not supported and will be skipped.
 */
struct JointSnapshot
{
  double position = 0.0;  ///< d->qpos[m->jnt_qposadr[id]]
  double velocity = 0.0;  ///< d->qvel[m->jnt_dofadr[id]]
  double effort   = 0.0;  ///< d->qfrc_actuator[m->jnt_dofadr[id]] (net actuator force/torque)
  /// d->ctrl[aid] for every actuator whose transmission targets this joint, keyed by actuator name.
  std::unordered_map<std::string, double> actuator_ctrls;
};

/**
 * Full aggregated snapshot produced by one SimDataAggregator::compute() call.
 * Cameras are keyed by the ImagePublisher `namespace` attribute (without trailing slash).
 * Joints are keyed by joint name.
 */
struct SimSnapshot
{
  double sim_time = 0.0;
  std::unordered_map<std::string, CameraFrame>   cameras;  ///< camera namespace → frame
  std::unordered_map<std::string, JointSnapshot> joints;   ///< joint name → snapshot
  bool valid = false;
};

/**
 * Thread-safe in-process registry for sharing simulation data across MuJoCo plugins
 * without going through the ROS 2 DDS/network transport layer.
 *
 * Usage pattern
 * -------------
 *   - Each ImagePublisher pushes a CameraFrame every time it renders, keyed by its
 *     `namespace` plugin attribute (trailing slash stripped).
 *   - SimDataAggregator reads those frames plus joint state directly from mjData, then
 *     writes the result as a SimSnapshot keyed by its `instance_name` attribute.
 *   - External C++ code (in the same process / shared library) calls getSnapshot() to
 *     read the latest data, or registers a callback to be notified synchronously from
 *     the simulation thread.
 *
 * Thread safety
 * -------------
 *   Camera frames and snapshots each have a dedicated mutex.  Callbacks are fired while
 *   the snapshot lock is held — callers MUST NOT call back into SimDataRegistry from
 *   within the callback (would deadlock).
 */
class SimDataRegistry
{
public:
  /// Invoked from the sim thread each time a new snapshot is stored.
  /// Do NOT call SimDataRegistry methods from inside the callback.
  using SnapshotCallback = std::function<void(const SimSnapshot &)>;

  /// Singleton accessor — the instance lives for the duration of the process.
  static SimDataRegistry & instance()
  {
    static SimDataRegistry inst;
    return inst;
  }

  // ── ImagePublisher API ──────────────────────────────────────────────────────

  /**
   * Store the latest rendered frame for a camera.
   * Called from ImagePublisher::compute() on the simulation thread.
   * No-op if no SimDataAggregator has registered interest in @p key.
   */
  void updateCameraFrame(const std::string & key, CameraFrame frame)
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    frame.seq = ++frame_seqs_[key];
    camera_frames_[key] = std::move(frame);
  }

  /**
   * Retrieve the most-recently stored frame for a camera namespace.
   * @return false if no frame has been stored yet for @p key.
   */
  bool getCameraFrame(const std::string & key, CameraFrame & out) const
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    auto it = camera_frames_.find(key);
    if (it == camera_frames_.end() || !it->second.valid)
      return false;
    out = it->second;
    return true;
  }

  /**
   * Return the current sequence number for a camera without copying frame data.
   * Returns 0 if no frame has been stored yet.
   * This is a cheap integer read used by SimDataAggregator to avoid redundant
   * full-frame copies (900 KB per camera) on sim steps between ImagePublisher updates.
   */
  uint64_t getCameraFrameSeq(const std::string & key) const
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    auto it = frame_seqs_.find(key);
    return (it != frame_seqs_.end()) ? it->second : 0;
  }

  /**
   * ImagePublisher will skip all registry work for keys with no registered consumers,
   * so there is zero overhead when SimDataAggregator is not present in the XML.
   */
  void registerCameraConsumer(const std::string & key)
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    camera_consumers_[key]++;
  }

  /// Called by SimDataAggregator on destruction to release interest in a camera namespace.
  void unregisterCameraConsumer(const std::string & key)
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    auto it = camera_consumers_.find(key);
    if (it != camera_consumers_.end())
    {
      if (--it->second <= 0)
      {
        camera_consumers_.erase(it);
        camera_frames_.erase(key);  // free memory too
      }
    }
  }

  /**
   * Returns true if at least one SimDataAggregator has registered interest in @p key.
   * ImagePublisher calls this to decide whether to flip and copy the color buffer.
   */
  bool hasCameraConsumer(const std::string & key) const
  {
    std::lock_guard<std::mutex> lk(camera_mutex_);
    auto it = camera_consumers_.find(key);
    return it != camera_consumers_.end() && it->second > 0;
  }

  // ── SimDataAggregator API ───────────────────────────────────────────────────

  /**
   * Store an aggregated snapshot.  Fires the registered callback (if any) while
   * the snapshot lock is held — see SnapshotCallback note above.
   */
  void updateSnapshot(const std::string & name, SimSnapshot snap)
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    snapshots_[name] = std::move(snap);
    auto it = callbacks_.find(name);
    if (it != callbacks_.end())
      it->second(snapshots_[name]);
  }

  /**
   * Retrieve the most-recently stored snapshot.
   * @return false if no snapshot has been stored yet for @p name.
   */
  bool getSnapshot(const std::string & name, SimSnapshot & out) const
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    auto it = snapshots_.find(name);
    if (it == snapshots_.end() || !it->second.valid)
      return false;
    out = it->second;
    return true;
  }

  /**
   * Register a callback invoked every time the snapshot named @p name is refreshed.
   * Pass an empty/null std::function to clear a previously registered callback.
   */
  void setSnapshotCallback(const std::string & name, SnapshotCallback cb)
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    if (cb)
      callbacks_[name] = std::move(cb);
    else
      callbacks_.erase(name);
  }

private:
  SimDataRegistry() = default;

  mutable std::mutex                                camera_mutex_;
  std::unordered_map<std::string, CameraFrame>      camera_frames_;
  std::unordered_map<std::string, int>              camera_consumers_;  ///< refcount per key
  std::unordered_map<std::string, uint64_t>         frame_seqs_;        ///< seq counter per key

  mutable std::mutex                                snapshot_mutex_;
  std::unordered_map<std::string, SimSnapshot>      snapshots_;
  std::unordered_map<std::string, SnapshotCallback> callbacks_;
};

}  // namespace MujocoRosUtils
