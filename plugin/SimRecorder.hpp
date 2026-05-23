#pragma once

#include "SimDataRegistry.hpp"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace MujocoRosUtils
{

/**
 * SimRecorder — records SimSnapshot data to disk in two formats:
 *
 *   MCAP  : Standard rosbag2/MCAP bag.  Topics written:
 *             /sim/{name}/cameras/{cam_ns}/color  — sensor_msgs/Image  (BGR8)
 *             /sim/{name}/joint_states            — sensor_msgs/JointState (pos/vel/eff)
 *             /sim/{name}/joint_commands          — sensor_msgs/JointState (ctrl values)
 *
 *   LeRobot v2.1 : HuggingFace LeRobot dataset layout:
 *             videos/chunk-000/observation.images.{CAM}/episode_XXXXXX.mp4
 *             data/chunk-000/episode_XXXXXX.jsonl   (→ .parquet via Python script)
 *             meta/info.json, episodes.jsonl, tasks.jsonl, episodes_stats.jsonl
 *
 * Call start() → addFrame() per sim step → stop().
 * stop() blocks until all I/O (including the Python Parquet conversion) is done.
 */
class SimRecorder
{
public:
  enum class Format
  {
    MCAP,
    LeRobot,
    Both
  };

  SimRecorder(std::string              aggregator_name,
              std::vector<std::string> camera_namespaces,
              std::vector<std::string> joint_names);
  ~SimRecorder();

  /**
   * Begin a recording.
   * @param output_dir  Parent directory; sub-directories are created inside.
   * @param format      MCAP, LeRobot, or Both.
   * @param episode_idx LeRobot episode index (ignored for MCAP-only).
   * @param task        Human-readable task description for LeRobot metadata.
   * @return true on success.
   */
  bool start(const std::string & output_dir,
             Format              format,
             int                 episode_idx = 0,
             const std::string & task        = "sim_task");

  /**
   * Finalize and flush everything.
   * @param frames_out   Out-param: number of frames recorded.
   * @param duration_out Out-param: wall-clock seconds since start().
   * @param discard      If true, delete all written files without saving.
   * @return true on success (JSONL→Parquet conversion failure is a warning, not an error).
   */
  bool stop(int & frames_out, double & duration_out, bool discard = false);

  /**
   * Record one snapshot.  Silently no-ops when not recording.
   * Must be called from the simulation thread (same thread as compute()).
   */
  void addFrame(const SimSnapshot & snap);

  bool isRecording() const { return recording_.load(); }
  int  episodeIdx()  const { return episode_idx_; }

private:
  // ── Config (set at construction) ───────────────────────────────────────────
  std::string              aggregator_name_;
  std::vector<std::string> camera_namespaces_;
  std::vector<std::string> joint_names_;

  // ── NVENC probe: checked once before first recording, then cached ──────────
  // -1 = not yet checked, 0 = unavailable (use libx264), 1 = available
  int  nvenc_available_ = -1;
  void probeNvenc();  ///< sets nvenc_available_ to 0 or 1

  // ── Recording state ─────────────────────────────────────────────────────────
  std::atomic<bool> recording_{false};
  Format            active_format_;
  std::string       output_dir_;
  int               episode_idx_   = 0;
  std::string       task_desc_;
  int               frame_count_         = 0;  ///< sim steps recorded (for stop response)
  int               video_frame_count_   = 0;  ///< actual video/state rows written
  double            start_sim_time_  = 0.0;
  double            last_sim_time_   = 0.0;
  std::chrono::steady_clock::time_point start_wall_;

  // ── MCAP ────────────────────────────────────────────────────────────────────
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> bag_writer_;
  rclcpp::Serialization<sensor_msgs::msg::Image>          img_ser_;
  rclcpp::Serialization<sensor_msgs::msg::JointState>     js_ser_;

  void openMcap(const std::string & bag_path);
  void writeMcapFrame(const SimSnapshot & snap);
  void closeMcap();

  std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  bagMsg(rclcpp::SerializedMessage & ser, const std::string & topic, int64_t ts_ns);

  // ── LeRobot ─────────────────────────────────────────────────────────────────
  std::string  lerobot_dir_;
  int          video_w_   = 0;
  int          video_h_   = 0;
  int  total_committed_episodes_ = 0;
  int  total_committed_frames_   = 0;

  /**
   * Per-camera background encoder.
   *
   * The simulation thread pushes raw BGR bytes into the queue (cheap memcpy).
   * A dedicated thread drains the queue and writes to an ffmpeg pipe using
   * h264_nvenc (GPU) — keeping the sim loop completely unblocked by encoding.
   *
   * An empty vector in the queue is the stop sentinel.
   */
  struct AsyncEncoder
  {
    FILE*                                pipe = nullptr;  ///< ffmpeg stdin pipe
    std::string                          path;            ///< output .mp4 file path
    std::queue<std::vector<uint8_t>>     q;               ///< raw BGR bytes; empty = sentinel
    std::mutex                           mtx;
    std::condition_variable              cv_;
    std::thread                          thread;
    std::atomic<bool>                    abort_flag{false};  ///< drain without encoding
    std::atomic<bool>                    encoder_ok{true};   ///< cleared on pipe/ffmpeg error
    std::atomic<int>                     frames_written{0};

    AsyncEncoder() = default;
    AsyncEncoder(const AsyncEncoder &) = delete;
    AsyncEncoder & operator=(const AsyncEncoder &) = delete;
  };
  std::unordered_map<std::string, std::unique_ptr<AsyncEncoder>> encoders_;
  std::unordered_map<std::string, uint64_t> last_video_seq_;  ///< dedup: last CameraFrame seq

  struct EpisodeRow
  {
    std::vector<float> obs_state;   ///< joint positions  [n_joints]
    std::vector<float> action;      ///< joint ctrl values [n_joints]
    float              timestamp = 0.0f;
    int                frame_index = 0;
  };
  std::vector<EpisodeRow> episode_rows_;

  /// Per-feature statistics (min/max/mean/std over all frames in one episode).
  struct FeatureStats
  {
    std::vector<float> min_vals;
    std::vector<float> max_vals;
    std::vector<float> mean;
    std::vector<float> std_dev;
  };
  struct EpisodeStats
  {
    FeatureStats obs_state;
    FeatureStats action;
    FeatureStats timestamp;   ///< single element
  };

  bool openLeRobot(const std::string & lr_dir);
  void writeLeRobotFrame(const SimSnapshot & snap);
  bool closeLeRobot();
  void discardLeRobot();
  void discardMcap();

  void startEncoder(const std::string & ns, const std::string & path, int w, int h);
  void flushEncoders();
  void discardEncoders();

  EpisodeStats computeEpisodeStats() const;
  void         writeLeRobotMeta(int total_frames, float fps, const EpisodeStats & stats);
  bool         convertJsonlToParquet(const std::string & jsonl_path,
                                     const std::string & parquet_path) const;
  std::string  episodeTag() const;
  std::string  chunkDir(const std::string & lr_root, const std::string & sub) const;
};

}  // namespace MujocoRosUtils
