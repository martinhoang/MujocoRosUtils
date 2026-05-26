#include "SimRecorder.hpp"
#include "mujoco_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcutils/allocator.h>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <pthread.h>
#include <sstream>

namespace fs = std::filesystem;

/// Select the position-actuator ctrl for a joint.
///
/// Each joint may have multiple actuators (e.g. a `_position` and a `_velocity`
/// actuator for ros2_control group-based switching).  `d->ctrl` for a MuJoCo
/// `<position>` actuator is the **target joint angle** (rad) — exactly what we
/// want to record as the action.  `d->ctrl` for a `<velocity>` actuator is a
/// target speed and must NOT be used as the action.
///
/// Strategy: prefer the actuator whose name ends with "_position"; fall back to
/// the first entry (for joints with only one actuator, e.g. hand joints).
static double pickPositionCtrl(
  const std::unordered_map<std::string, double> & ctrls,
  const std::string & joint_name)
{
  if (ctrls.empty())
    return std::numeric_limits<double>::quiet_NaN();

  // Preferred key: "<joint_name>_position"
  const std::string preferred = joint_name + "_position";
  auto it = ctrls.find(preferred);
  if (it != ctrls.end())
    return it->second;

  // Fallback: any key that ends with "_position"
  for (const auto & [key, val] : ctrls)
    if (key.size() > 9 && key.compare(key.size() - 9, 9, "_position") == 0)
      return val;

  // Last resort: first entry (single-actuator joints, e.g. hand)
  return ctrls.begin()->second;
}

/// Fixed video frame rate used for both the VideoWriter and LeRobot metadata.
/// State/action rows are only written when a new camera frame is available,
/// so tabular fps matches video fps automatically.
static constexpr float VIDEO_FPS = 30.0f;

// Video encoding quality (CQ/CRF scale: 0=lossless, 18=near-lossless, 23=default).
// Training data should use high quality to minimise the visual gap between sim
// recordings and real-camera inference.  Lower = better quality, larger file.
static constexpr int VIDEO_CQ_NVENC = 18;  ///< NVENC constant-quality target (h264_nvenc -cq)
static constexpr int VIDEO_CRF_X264 = 15;  ///< libx264 CRF (fallback)

namespace MujocoRosUtils
{

// ── helpers ──────────────────────────────────────────────────────────────────

static std::string zeroPad(int v, int w)
{
  std::ostringstream ss;
  ss << std::setw(w) << std::setfill('0') << v;
  return ss.str();
}

// JSON-escape a string (only handles the characters we actually produce).
static std::string jsonStr(const std::string & s)
{
  return "\"" + s + "\"";
}

static std::string floatVecToJson(const std::vector<float> & v)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << "[";
  for (size_t i = 0; i < v.size(); ++i)
  {
    if (i)
      ss << ",";
    // JSON does not support NaN or Inf — write 0.0 as a safe fallback.
    // NaN in action means the joint has no actuator (passive joint);
    // 0.0 is a neutral placeholder that won't break downstream parsers.
    const float val = v[i];
    ss << (std::isfinite(val) ? val : 0.0f);
  }
  ss << "]";
  return ss.str();
}

// ── Constructor / Destructor ─────────────────────────────────────────────────

SimRecorder::SimRecorder(std::string              aggregator_name,
                         std::vector<std::string> camera_namespaces,
                         std::vector<std::string> joint_names)
    : aggregator_name_(std::move(aggregator_name))
    , camera_namespaces_(std::move(camera_namespaces))
    , joint_names_(std::move(joint_names))
{}

SimRecorder::~SimRecorder()
{
  if (recording_.load())
  {
    int    f;
    double d;
    stop(f, d);
  }
}

// ── Public API ───────────────────────────────────────────────────────────────

bool SimRecorder::start(const std::string & output_dir,
                        Format              format,
                        int                 episode_idx,
                        const std::string & task)
{
  if (recording_.load())
  {
    print_warning("[SimRecorder] start() called while already recording — ignored.\n");
    return false;
  }

  active_format_  = format;
  output_dir_     = output_dir;
  episode_idx_    = episode_idx;
  task_desc_      = task.empty() ? "sim_task" : task;
  frame_count_    = 0;
  start_sim_time_ = -1.0;
  last_sim_time_  = 0.0;
  start_wall_     = std::chrono::steady_clock::now();

  try
  {
    fs::create_directories(output_dir_);

    if (format == Format::MCAP || format == Format::Both || format == Format::HDF5AndMCAP)
    {
      std::string bag_path = (fs::path(output_dir_) / "bag").string();
      openMcap(bag_path);
    }

    if (format == Format::LeRobot || format == Format::Both || format == Format::HDF5AndLeRobot)
    {
      lerobot_dir_ = (fs::path(output_dir_) / "lerobot").string();
      if (!openLeRobot(lerobot_dir_))
        return false;
      // Always use the committed episode count as the floor so restarts never
      // overwrite episodes from a previous session.
      episode_idx_ = std::max(episode_idx_, total_committed_episodes_);
    }

    if (format == Format::HDF5 || format == Format::HDF5AndMCAP || format == Format::HDF5AndLeRobot)
    {
      const std::string hdf5_dir = (fs::path(output_dir_) / "hdf5").string();
      fs::create_directories(hdf5_dir);
      hdf5_path_ = (fs::path(hdf5_dir) / ("episode_" + zeroPad(episode_idx_, 6) + ".hdf5")).string();
      if (!openHDF5(hdf5_path_))
        return false;
    }
  }
  catch (const std::exception & e)
  {
    print_error("[SimRecorder] start() failed: %s\n", e.what());
    return false;
  }

  recording_.store(true);

  const char * fmt_str =
    format == Format::MCAP            ? "mcap"
    : format == Format::LeRobot       ? "lerobot"
    : format == Format::Both          ? "both"
    : format == Format::HDF5          ? "hdf5"
    : format == Format::HDF5AndMCAP   ? "hdf5+mcap"
    :                                   "hdf5+lerobot";

  print_confirm("[SimRecorder] Recording started → %s  (format=%s, episode=%d)\n",
                output_dir_.c_str(), fmt_str, episode_idx_);
  return true;
}

bool SimRecorder::stop(int & frames_out, double & duration_out, bool discard)
{
  if (!recording_.load())
  {
    print_warning("[SimRecorder] stop() called while not recording — ignored.\n");
    frames_out   = 0;
    duration_out = 0.0;
    return false;
  }

  recording_.store(false);
  frames_out   = discard ? 0 : frame_count_;
  duration_out = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start_wall_)
                   .count();

  bool ok = true;

  if (discard)
  {
    if (active_format_ == Format::MCAP || active_format_ == Format::Both ||
        active_format_ == Format::HDF5AndMCAP)
      discardMcap();
    if (active_format_ == Format::LeRobot || active_format_ == Format::Both ||
        active_format_ == Format::HDF5AndLeRobot)
      discardLeRobot();
    if (active_format_ == Format::HDF5 || active_format_ == Format::HDF5AndMCAP ||
        active_format_ == Format::HDF5AndLeRobot)
      discardHDF5();
    print_confirm("[SimRecorder] Recording discarded — episode %d removed.\n", episode_idx_);
  }
  else
  {
    if (active_format_ == Format::MCAP || active_format_ == Format::Both ||
        active_format_ == Format::HDF5AndMCAP)
      closeMcap();
    if (active_format_ == Format::LeRobot || active_format_ == Format::Both ||
        active_format_ == Format::HDF5AndLeRobot)
      ok = closeLeRobot() && ok;
    if (active_format_ == Format::HDF5 || active_format_ == Format::HDF5AndMCAP ||
        active_format_ == Format::HDF5AndLeRobot)
      ok = closeHDF5() && ok;
    print_confirm("[SimRecorder] Recording stopped — %d frames, %.2f s.\n",
                  frames_out, duration_out);
  }

  return ok;
}

void SimRecorder::addFrame(const SimSnapshot & snap)
{
  if (!recording_.load())
    return;

  if (start_sim_time_ < 0.0)
    start_sim_time_ = snap.sim_time;
  last_sim_time_ = snap.sim_time;
  ++frame_count_;

  if (active_format_ == Format::MCAP || active_format_ == Format::Both ||
      active_format_ == Format::HDF5AndMCAP)
    writeMcapFrame(snap);

  if (active_format_ == Format::LeRobot || active_format_ == Format::Both ||
      active_format_ == Format::HDF5AndLeRobot)
    writeLeRobotFrame(snap);

  if (active_format_ == Format::HDF5 || active_format_ == Format::HDF5AndMCAP ||
      active_format_ == Format::HDF5AndLeRobot)
    writeHDF5Frame(snap);
}

// ══════════════════════════════════════════════════════════════════════════════
//  MCAP
// ══════════════════════════════════════════════════════════════════════════════

void SimRecorder::openMcap(const std::string & bag_path)
{
  bag_writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri        = bag_path;
  storage_opts.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions conv_opts;
  conv_opts.input_serialization_format  = "cdr";
  conv_opts.output_serialization_format = "cdr";

  bag_writer_->open(storage_opts, conv_opts);

  // Register topics
  auto addTopic = [&](const std::string & name, const std::string & type) {
    rosbag2_storage::TopicMetadata tm;
    tm.name                 = name;
    tm.type                 = type;
    tm.serialization_format = "cdr";
    bag_writer_->create_topic(tm);
  };

  for (const auto & ns : camera_namespaces_)
    addTopic("/sim/" + aggregator_name_ + "/cameras/" + ns + "/color",
             "sensor_msgs/msg/Image");

  addTopic("/sim/" + aggregator_name_ + "/joint_states",   "sensor_msgs/msg/JointState");
  addTopic("/sim/" + aggregator_name_ + "/joint_commands", "sensor_msgs/msg/JointState");
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
SimRecorder::bagMsg(rclcpp::SerializedMessage & ser,
                    const std::string &          topic,
                    int64_t                      ts_ns)
{
  // Safe-copy the serialized buffer into a new rcutils_uint8_array_t owned by the bag message.
  auto & rcl_msg = ser.get_rcl_serialized_message();
  auto   alloc   = rcutils_get_default_allocator();

  auto arr        = new rcutils_uint8_array_t{};
  arr->allocator  = alloc;
  arr->buffer_length   = rcl_msg.buffer_length;
  arr->buffer_capacity = rcl_msg.buffer_length;
  arr->buffer = static_cast<uint8_t *>(alloc.allocate(rcl_msg.buffer_length, alloc.state));
  std::memcpy(arr->buffer, rcl_msg.buffer, rcl_msg.buffer_length);

  auto bag = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag->topic_name      = topic;
  bag->send_timestamp      = static_cast<rcutils_time_point_value_t>(ts_ns);
  bag->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(arr,
      [](rcutils_uint8_array_t * p) {
        if (p->buffer && p->allocator.deallocate)
          p->allocator.deallocate(p->buffer, p->allocator.state);
        delete p;
      });

  return bag;
}

void SimRecorder::writeMcapFrame(const SimSnapshot & snap)
{
  const auto ts_ns = static_cast<int64_t>(snap.sim_time * 1e9);

  // ── Camera images ──────────────────────────────────────────────────────────
  for (const auto & ns : camera_namespaces_)
  {
    auto it = snap.cameras.find(ns);
    if (it == snap.cameras.end() || !it->second.valid)
      continue;

    const auto & frame = it->second;
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp.sec     = static_cast<int32_t>(snap.sim_time);
    img_msg.header.stamp.nanosec = static_cast<uint32_t>((snap.sim_time - img_msg.header.stamp.sec) * 1e9);
    img_msg.header.frame_id      = ns;
    img_msg.height               = static_cast<uint32_t>(frame.height);
    img_msg.width                = static_cast<uint32_t>(frame.width);
    img_msg.encoding             = "bgr8";
    img_msg.step                 = static_cast<uint32_t>(frame.width * 3);
    img_msg.data                 = frame.data;

    rclcpp::SerializedMessage ser;
    img_ser_.serialize_message(&img_msg, &ser);
    bag_writer_->write(bagMsg(ser, "/sim/" + aggregator_name_ + "/cameras/" + ns + "/color", ts_ns));
  }

  // ── Joint states ──────────────────────────────────────────────────────────
  sensor_msgs::msg::JointState js_state;
  sensor_msgs::msg::JointState js_cmd;
  js_state.header.stamp.sec     = static_cast<int32_t>(snap.sim_time);
  js_state.header.stamp.nanosec = static_cast<uint32_t>((snap.sim_time - js_state.header.stamp.sec) * 1e9);
  js_cmd.header = js_state.header;

  for (const auto & jname : joint_names_)
  {
    auto it = snap.joints.find(jname);
    if (it == snap.joints.end())
      continue;

    const auto & js = it->second;
    js_state.name.push_back(jname);
    js_state.position.push_back(js.position);
    js_state.velocity.push_back(js.velocity);
    js_state.effort.push_back(js.effort);

    // Commands: prefer the position actuator ctrl (target angle, rad).
    // Each arm joint has both a _position and a _velocity actuator — we must
    // pick the _position one; using begin() on unordered_map is non-deterministic.
    double cmd = pickPositionCtrl(js.actuator_ctrls, jname);
    js_cmd.name.push_back(jname);
    js_cmd.position.push_back(cmd);
    js_cmd.velocity.push_back(std::numeric_limits<double>::quiet_NaN());
    js_cmd.effort.push_back(std::numeric_limits<double>::quiet_NaN());
  }

  if (!js_state.name.empty())
  {
    rclcpp::SerializedMessage ser_s, ser_c;
    js_ser_.serialize_message(&js_state, &ser_s);
    js_ser_.serialize_message(&js_cmd,   &ser_c);
    bag_writer_->write(bagMsg(ser_s, "/sim/" + aggregator_name_ + "/joint_states",   ts_ns));
    bag_writer_->write(bagMsg(ser_c, "/sim/" + aggregator_name_ + "/joint_commands", ts_ns));
  }
}

void SimRecorder::closeMcap()
{
  if (bag_writer_)
  {
    bag_writer_.reset();
    print_confirm("[SimRecorder] MCAP bag closed.\n");
  }
}

// ══════════════════════════════════════════════════════════════════════════════
//  LeRobot v2
// ══════════════════════════════════════════════════════════════════════════════

std::string SimRecorder::episodeTag() const
{
  return "episode_" + zeroPad(episode_idx_, 6);
}

std::string SimRecorder::chunkDir(const std::string & lr_root,
                                  const std::string & sub) const
{
  return (fs::path(lr_root) / sub / "chunk-000").string();
}

bool SimRecorder::openLeRobot(const std::string & lr_dir)
{
  lerobot_dir_ = lr_dir;
  episode_rows_.clear();
  encoders_.clear();
  last_video_seq_.clear();
  video_w_ = video_h_ = 0;
  video_frame_count_ = 0;

  // Create directory tree up-front; encoders are started on first frame
  // (because we don't know image dimensions yet).
  fs::create_directories(chunkDir(lr_dir, "data"));
  for (const auto & ns : camera_namespaces_)
    fs::create_directories(chunkDir(lr_dir, "videos") + "/observation.images." + ns);
  fs::create_directories(fs::path(lr_dir) / "meta");

  // Read existing info.json (if any) to resume committed episode/frame counts.
  total_committed_episodes_ = 0;
  total_committed_frames_   = 0;
  const std::string info_path = (fs::path(lr_dir) / "meta" / "info.json").string();
  if (fs::exists(info_path))
  {
    std::ifstream f(info_path);
    std::string   content((std::istreambuf_iterator<char>(f)), {});
    auto parseIntField = [&](const std::string & key) -> int {
      const std::string needle = "\"" + key + "\"";
      auto pos = content.find(needle);
      if (pos == std::string::npos) return 0;
      pos = content.find(':', pos + needle.size());
      if (pos == std::string::npos) return 0;
      try { return std::stoi(content.substr(pos + 1)); } catch (...) { return 0; }
    };
    total_committed_episodes_ = parseIntField("total_episodes");
    total_committed_frames_   = parseIntField("total_frames");
    print_info("[SimRecorder] Resuming dataset: %d prior episodes, %d prior frames\n",
               total_committed_episodes_, total_committed_frames_);
  }

  // Early check: warn if pyarrow is missing (Parquet conversion will fail at stop()).
  if (std::system("python3 -c 'import pyarrow' 2>/dev/null") != 0 &&
      std::system("python3 -c 'import polars'  2>/dev/null") != 0)
  {
    print_warning("[SimRecorder] WARNING: neither pyarrow nor polars found — "
                  "Parquet conversion will fail.  Run: pip install pyarrow\n");
  }

  return true;
}

void SimRecorder::writeLeRobotFrame(const SimSnapshot & snap)
{
  const float rel_time = static_cast<float>(snap.sim_time - start_sim_time_);

  // ── Video frames ───────────────────────────────────────────────────────────
  // Gate on CameraFrame::seq to skip duplicate frames between ImagePublisher
  // publish steps (ImagePublisher runs every publish_skip_ sim steps; the cache
  // returns the same frame for all intermediate steps).
  bool new_video_frame = false;

  for (const auto & ns : camera_namespaces_)
  {
    auto cit = snap.cameras.find(ns);
    if (cit == snap.cameras.end() || !cit->second.valid)
      continue;

    const auto & frame = cit->second;

    // Skip if this is the same frame we already encoded last step.
    auto & last_seq = last_video_seq_[ns];
    if (frame.seq == last_seq)
      continue;
    last_seq = frame.seq;

    // Lazy encoder open (dimensions known only on first frame).
    if (encoders_.find(ns) == encoders_.end())
    {
      video_w_ = frame.width;
      video_h_ = frame.height;
      std::string vdir  = chunkDir(lerobot_dir_, "videos") + "/observation.images." + ns;
      std::string vpath = (fs::path(vdir) / (episodeTag() + ".mp4")).string();
      startEncoder(ns, vpath, frame.width, frame.height);
    }

    auto eit = encoders_.find(ns);
    if (eit == encoders_.end())
      continue;

    // Early encoder failure detection.
    if (!eit->second->encoder_ok.load())
    {
      print_error("[SimRecorder] Encoder failure on camera '%s' — stopping recording.\n",
                  ns.c_str());
      recording_.store(false);
      return;
    }

    // Push raw BGR bytes to the encoder queue (non-blocking).
    std::vector<uint8_t> raw(frame.data.begin(), frame.data.end());
    {
      std::lock_guard<std::mutex> lk(eit->second->mtx);
      eit->second->q.push(std::move(raw));
    }
    eit->second->cv_.notify_one();
    new_video_frame = true;
  }

  // Skip tabular accumulation on sim steps without a new camera frame.
  // When there are no cameras, always accumulate.
  if (!camera_namespaces_.empty() && !new_video_frame)
    return;

  const int fidx = video_frame_count_++;

  // ── Tabular row ────────────────────────────────────────────────────────────
  EpisodeRow row;
  row.timestamp   = rel_time;
  row.frame_index = fidx;

  for (const auto & jname : joint_names_)
  {
    auto jit = snap.joints.find(jname);
    float pos = 0.0f, ctrl = 0.0f;
    if (jit != snap.joints.end())
    {
      pos  = static_cast<float>(jit->second.position);
      double raw_ctrl = pickPositionCtrl(jit->second.actuator_ctrls, jname);
      // For joints with no actuator (passive/underactuated), pickPositionCtrl
      // returns NaN.  Fall back to the actual joint position — the joint is
      // not commanded but is being held by mechanical coupling, so its current
      // position is the best available proxy for the "action".
      ctrl = std::isfinite(raw_ctrl) ? static_cast<float>(raw_ctrl)
                                     : pos;
    }
    row.obs_state.push_back(pos);
    row.action.push_back(ctrl);
  }

  episode_rows_.push_back(std::move(row));
}

bool SimRecorder::closeLeRobot()
{
  // Block until all ffmpeg encoder threads have flushed and closed the MP4 files.
  flushEncoders();

  if (episode_rows_.empty())
  {
    print_warning("[SimRecorder] LeRobot episode has zero rows — skipping write.\n");
    return true;
  }

  // Global frame offset — rows written before this episode.
  const int global_offset = total_committed_frames_;

  // ── Write JSONL ────────────────────────────────────────────────────────────
  std::string data_dir  = chunkDir(lerobot_dir_, "data");
  std::string jsonl_path = (fs::path(data_dir) / (episodeTag() + ".jsonl")).string();
  {
    std::ofstream out(jsonl_path);
    if (!out)
    {
      print_error("[SimRecorder] Cannot open %s for writing.\n", jsonl_path.c_str());
      return false;
    }

    const int n_total = static_cast<int>(episode_rows_.size());
    for (int i = 0; i < n_total; ++i)
    {
      const auto & row = episode_rows_[i];
      out << "{"
          << jsonStr("observation.state") << ":" << floatVecToJson(row.obs_state) << ","
          << jsonStr("action")            << ":" << floatVecToJson(row.action)    << ","
          << jsonStr("timestamp")         << ":" << std::fixed << std::setprecision(6) << row.timestamp  << ","
          << jsonStr("frame_index")       << ":" << row.frame_index  << ","
          << jsonStr("episode_index")     << ":" << episode_idx_     << ","
          << jsonStr("index")             << ":" << (global_offset + row.frame_index) << ","
          << jsonStr("task_index")        << ":0,"
          << jsonStr("next.done")         << ":" << (i == n_total - 1 ? "true" : "false")
          << "}\n";
    }
  }

  // ── Write meta files ───────────────────────────────────────────────────────
  const float fps   = VIDEO_FPS;
  const EpisodeStats stats = computeEpisodeStats();
  writeLeRobotMeta(static_cast<int>(episode_rows_.size()), fps, stats);

  // ── Convert JSONL → Parquet (background — doesn't block episode commit) ─────
  std::string parquet_path = jsonl_path.substr(0, jsonl_path.size() - 6) + ".parquet";
  const size_t n_rows    = episode_rows_.size();
  const std::string ldir = lerobot_dir_;

  std::thread([this, jsonl_path, parquet_path, n_rows, fps, ldir]() {
    bool converted = convertJsonlToParquet(jsonl_path, parquet_path);
    if (converted)
    {
      print_confirm("[SimRecorder] LeRobot episode written to %s  (%zu frames, fps~%.1f)\n",
                    ldir.c_str(), n_rows, fps);
    }
    else
    {
      print_warning("[SimRecorder] Parquet conversion failed.  "
                    "The raw JSONL is at: %s\n"
                    "  Convert manually: python3 -c \""
                    "import json,pyarrow as pa,pyarrow.parquet as pq;"
                    "rows=[json.loads(l) for l in open('%s')];"
                    "pq.write_table(pa.Table.from_pylist(rows),'%s')\"\n",
                    jsonl_path.c_str(), jsonl_path.c_str(), parquet_path.c_str());
    }
  }).detach();
  episode_rows_.clear();
  return true;
}

void SimRecorder::discardLeRobot()
{
  discardEncoders();
  last_video_seq_.clear();
  episode_rows_.clear();
  video_frame_count_ = 0;
  print_confirm("[SimRecorder] LeRobot episode %d discarded.\n", episode_idx_);
}

void SimRecorder::discardMcap()
{
  if (bag_writer_)
  {
    bag_writer_.reset();
    const std::string bag_path = (fs::path(output_dir_) / "bag").string();
    if (fs::exists(bag_path))
      fs::remove_all(bag_path);
  }
}

// ── Async ffmpeg encoder helpers ─────────────────────────────────────────────

// Path to the ffmpeg binary used for encoding.
// If /opt/ffmpeg-nvenc exists (BtbN NVENC-capable build installed separately
// from the system ffmpeg so rerun is not broken), use it.  Otherwise fall back
// to the system ffmpeg on PATH.
static std::string ffmpegBinary()
{
  static const std::string nvenc_bin = "/opt/ffmpeg-nvenc";
  return fs::exists(nvenc_bin) ? nvenc_bin : "ffmpeg";
}

void SimRecorder::probeNvenc()
{
  const std::string ff = ffmpegBinary();
  // Run a zero-frame encode using h264_nvenc.  If ffmpeg exits with code 0
  // (or the only failure is "no frames"), NVENC is available.
  // We redirect stderr so the probe is silent in the terminal.
  std::string cmd = ff +
    " -y -f lavfi -i nullsrc=s=64x64:r=1:d=0 "
    "-frames:v 1 -c:v h264_nvenc -f null /dev/null "
    ">/dev/null 2>&1";
  int rc = std::system(cmd.c_str());
  nvenc_available_ = (rc == 0) ? 1 : 0;
  print_info("[SimRecorder] NVENC probe (%s): %s\n",
             ff.c_str(),
             nvenc_available_ ? "available (GPU encoding)" : "unavailable — using libx264");
}

void SimRecorder::startEncoder(const std::string & ns, const std::string & path,
                               int w, int h)
{
  // Probe NVENC once per SimRecorder lifetime.
  if (nvenc_available_ < 0)
    probeNvenc();

  auto enc = std::make_unique<AsyncEncoder>();
  enc->path = path;

  auto tryOpen = [&](const std::string & codec_opts) -> FILE * {
    std::string cmd = ffmpegBinary() +
      " -y -f rawvideo -pix_fmt bgr24"
      " -s " + std::to_string(w) + "x" + std::to_string(h) +
      " -r " + std::to_string(static_cast<int>(VIDEO_FPS)) +
      " -i pipe:0 " + codec_opts +
      " \"" + path + "\" 2>/tmp/ffmpeg_simrec.log";
    return popen(cmd.c_str(), "w");
  };

  // Use NVENC (GPU) if probed available, else libx264.
  // Quality-based encoding (CQ/CRF) is preferred over fixed-bitrate for training
  // data: it guarantees uniform perceptual quality regardless of scene complexity
  // rather than sacrificing detail on slow-moving scenes to hit a bitrate target.
  std::string codec_used;
  if (nvenc_available_)
  {
    codec_used = "h264_nvenc";
    enc->pipe = tryOpen("-c:v h264_nvenc -rc vbr -cq " + std::to_string(VIDEO_CQ_NVENC) +
                        " -preset p4 -pix_fmt yuv420p");
    if (!enc->pipe)
    {
      // popen() itself failed (very unusual); fall through to x264.
      print_warning("[SimRecorder] popen(h264_nvenc) failed, falling back to libx264\n");
      nvenc_available_ = 0;
    }
  }
  if (!nvenc_available_ || !enc->pipe)
  {
    codec_used = "libx264";
    // ultrafast preset minimises CPU time in the background drain thread.
    // For training data quality is set by CRF (not affected by preset).
    enc->pipe = tryOpen("-c:v libx264 -crf " + std::to_string(VIDEO_CRF_X264) +
                        " -preset ultrafast -pix_fmt yuv420p");
  }

  if (!enc->pipe)
  {
    print_error("[SimRecorder] Failed to open ffmpeg pipe for camera '%s' "
                "(check /tmp/ffmpeg_simrec.log)\n", ns.c_str());
    enc->encoder_ok.store(false);
    encoders_[ns] = std::move(enc);
    return;
  }

  // Encoder background thread: drains queue → fwrite to ffmpeg stdin.
  enc->thread = std::thread([e = enc.get()]() {
    // Block SIGPIPE for this thread so that writing to a closed ffmpeg pipe
    // returns EPIPE via fwrite (errno) instead of killing the whole process.
    // This is the root cause of exit code -13 (SIGPIPE) when ffmpeg exits early
    // (e.g. GPU encoder init failure) while the main thread is still pushing frames.
    // We use pthread_sigmask (thread-local) rather than signal() (process-wide).
    {
      sigset_t mask;
      sigemptyset(&mask);
      sigaddset(&mask, SIGPIPE);
      pthread_sigmask(SIG_BLOCK, &mask, nullptr);
    }

    while (true)
    {
      std::unique_lock<std::mutex> lk(e->mtx);
      e->cv_.wait(lk, [e] { return !e->q.empty(); });
      std::vector<uint8_t> buf = std::move(e->q.front());
      e->q.pop();
      lk.unlock();

      if (buf.empty())  // sentinel — done
        break;

      if (!e->abort_flag.load())
      {
        if (fwrite(buf.data(), 1, buf.size(), e->pipe) != buf.size())
        {
          e->encoder_ok.store(false);
          // Drain remaining frames without writing (pipe is broken).
          e->abort_flag.store(true);
        }
        else
        {
          e->frames_written.fetch_add(1);
        }
      }
    }
    int rc = pclose(e->pipe);
    if (rc != 0 && e->encoder_ok.load())
    {
      e->encoder_ok.store(false);
      // Log ffmpeg's exit code to help diagnose GPU/codec failures.
      print_error("[SimRecorder] ffmpeg encoder exited with code %d — check /tmp/ffmpeg_simrec.log\n", rc);
    }
  });

  print_info("[SimRecorder] Encoder started for camera '%s' → %s  [%s, CQ/CRF=%d]\n",
             ns.c_str(), path.c_str(), codec_used.c_str(),
             (codec_used == "h264_nvenc") ? VIDEO_CQ_NVENC : VIDEO_CRF_X264);
  encoders_[ns] = std::move(enc);
}

void SimRecorder::flushEncoders()
{
  for (auto & kv : encoders_)
  {
    auto & enc = *kv.second;
    // Send empty-vector sentinel to signal end-of-stream.
    {
      std::lock_guard<std::mutex> lk(enc.mtx);
      enc.q.push({});
    }
    enc.cv_.notify_one();
  }
  // Join all threads (blocks while ffmpeg finalizes the MP4).
  for (auto & kv : encoders_)
  {
    auto & enc = *kv.second;
    if (enc.thread.joinable())
      enc.thread.join();
    if (!enc.encoder_ok.load())
      print_error("[SimRecorder] Encoder for '%s' reported an error — video may be corrupt.\n",
                  kv.first.c_str());
  }
  encoders_.clear();
}

void SimRecorder::discardEncoders()
{
  for (auto & kv : encoders_)
  {
    auto & enc = *kv.second;
    enc.abort_flag.store(true);
    // Drain current queue without encoding.
    {
      std::lock_guard<std::mutex> lk(enc.mtx);
      while (!enc.q.empty()) enc.q.pop();
      enc.q.push({});  // sentinel
    }
    enc.cv_.notify_one();
  }
  for (auto & kv : encoders_)
  {
    auto & enc = *kv.second;
    if (enc.thread.joinable())
      enc.thread.join();
    if (fs::exists(enc.path))
      fs::remove(enc.path);
  }
  encoders_.clear();
}

SimRecorder::EpisodeStats SimRecorder::computeEpisodeStats() const
{
  EpisodeStats es;
  if (episode_rows_.empty())
    return es;

  const int n        = static_cast<int>(episode_rows_.size());
  const int n_joints = static_cast<int>(joint_names_.size());

  auto initFS = [&](int dims) -> FeatureStats {
    FeatureStats fs;
    fs.min_vals.assign(dims,  std::numeric_limits<float>::max());
    fs.max_vals.assign(dims,  std::numeric_limits<float>::lowest());
    fs.mean.assign(dims,    0.0f);
    fs.std_dev.assign(dims, 0.0f);
    return fs;
  };

  es.obs_state = initFS(n_joints);
  es.action    = initFS(n_joints);
  es.timestamp = initFS(1);

  // First pass: min, max, running sum for mean.
  for (const auto & row : episode_rows_)
  {
    for (int j = 0; j < n_joints; ++j)
    {
      float v = (j < static_cast<int>(row.obs_state.size())) ? row.obs_state[j] : 0.0f;
      es.obs_state.min_vals[j] = std::min(es.obs_state.min_vals[j], v);
      es.obs_state.max_vals[j] = std::max(es.obs_state.max_vals[j], v);
      es.obs_state.mean[j]    += v;

      v = (j < static_cast<int>(row.action.size())) ? row.action[j] : 0.0f;
      es.action.min_vals[j] = std::min(es.action.min_vals[j], v);
      es.action.max_vals[j] = std::max(es.action.max_vals[j], v);
      es.action.mean[j]    += v;
    }
    float ts = row.timestamp;
    es.timestamp.min_vals[0] = std::min(es.timestamp.min_vals[0], ts);
    es.timestamp.max_vals[0] = std::max(es.timestamp.max_vals[0], ts);
    es.timestamp.mean[0]    += ts;
  }

  const float inv_n = 1.0f / static_cast<float>(n);
  for (int j = 0; j < n_joints; ++j)
  {
    es.obs_state.mean[j] *= inv_n;
    es.action.mean[j]    *= inv_n;
  }
  es.timestamp.mean[0] *= inv_n;

  // Second pass: variance accumulation.
  for (const auto & row : episode_rows_)
  {
    for (int j = 0; j < n_joints; ++j)
    {
      float d = ((j < static_cast<int>(row.obs_state.size())) ? row.obs_state[j] : 0.0f)
                - es.obs_state.mean[j];
      es.obs_state.std_dev[j] += d * d;

      d = ((j < static_cast<int>(row.action.size())) ? row.action[j] : 0.0f)
          - es.action.mean[j];
      es.action.std_dev[j] += d * d;
    }
    float d = row.timestamp - es.timestamp.mean[0];
    es.timestamp.std_dev[0] += d * d;
  }

  for (int j = 0; j < n_joints; ++j)
  {
    es.obs_state.std_dev[j] = std::sqrt(es.obs_state.std_dev[j] * inv_n);
    es.action.std_dev[j]    = std::sqrt(es.action.std_dev[j]    * inv_n);
  }
  es.timestamp.std_dev[0] = std::sqrt(es.timestamp.std_dev[0] * inv_n);

  return es;
}

void SimRecorder::writeLeRobotMeta(int total_frames, float fps,
                                   const EpisodeStats & stats)
{
  const std::string meta_dir = (fs::path(lerobot_dir_) / "meta").string();
  fs::create_directories(meta_dir);

  const int n_joints  = static_cast<int>(joint_names_.size());
  const int n_cameras = static_cast<int>(camera_namespaces_.size());

  // Accumulated totals: include all prior episodes + this one.
  const int new_total_episodes = total_committed_episodes_ + 1;
  const int new_total_frames   = total_committed_frames_   + total_frames;
  const int total_videos       = n_cameras * new_total_episodes;

  // ── info.json (overwrite with cumulative totals) ───────────────────────────
  {
    std::ofstream out(meta_dir + "/info.json");
    out << "{\n"
        << "  \"codebase_version\": \"v2.1\",\n"
        << "  \"fps\": " << static_cast<int>(fps + 0.5f) << ",\n"
        << "  \"robot_type\": \"mujoco\",\n"
        << "  \"total_episodes\": " << new_total_episodes << ",\n"
        << "  \"total_frames\": "   << new_total_frames   << ",\n"
        << "  \"total_tasks\": 1,\n"
        << "  \"total_chunks\": 1,\n"
        << "  \"total_videos\": " << total_videos << ",\n"
        << "  \"chunks_size\": 1000,\n"
        << "  \"splits\": {\"train\": \"0:" << new_total_episodes << "\"},\n"
        << "  \"data_path\": \"data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet\",\n"
        << "  \"video_path\": \"videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4\",\n"
        << "  \"features\": {\n";

    // observation.state
    out << "    \"observation.state\": {\n"
        << "      \"dtype\": \"float32\", \"shape\": [" << n_joints << "],\n"
        << "      \"names\": [";
    for (int i = 0; i < n_joints; ++i)
    {
      if (i)
        out << ", ";
      out << jsonStr(joint_names_[i]);
    }
    out << "]\n    },\n";

    // action
    out << "    \"action\": {\n"
        << "      \"dtype\": \"float32\", \"shape\": [" << n_joints << "],\n"
        << "      \"names\": [";
    for (int i = 0; i < n_joints; ++i)
    {
      if (i)
        out << ", ";
      out << jsonStr(joint_names_[i]);
    }
    out << "]\n    }";

    // camera video features
    for (int c = 0; c < n_cameras; ++c)
    {
      out << ",\n"
          << "    \"observation.images." << camera_namespaces_[c] << "\": {\n"
          << "      \"dtype\": \"video\",\n"
          << "      \"shape\": [" << video_h_ << ", " << video_w_ << ", 3],\n"
          << "      \"names\": [\"height\", \"width\", \"channel\"],\n"
          << "      \"info\": {\n"
          << "        \"video.fps\": " << static_cast<int>(fps + 0.5f) << ",\n"
          << "        \"video.codec\": \"avc1\",\n"
          << "        \"video.pix_fmt\": \"yuv420p\",\n"
          << "        \"video.is_depth_map\": false,\n"
          << "        \"has_audio\": false\n"
          << "      }\n"
          << "    }";
    }

    // scalar features
    for (const char * feat : {"timestamp", "frame_index", "episode_index", "index", "task_index"})
      out << ",\n    " << jsonStr(feat) << ": {\"dtype\": \"float32\", \"shape\": [1], \"names\": null}";
    out << ",\n    \"next.done\": {\"dtype\": \"bool\", \"shape\": [1], \"names\": null}\n";

    out << "  }\n}\n";
  }

  // ── episodes.jsonl (append — one line per episode) ─────────────────────────
  {
    std::ofstream out(meta_dir + "/episodes.jsonl", std::ios::app);
    out << "{\"episode_index\": " << episode_idx_
        << ", \"tasks\": [" << jsonStr(task_desc_) << "]"
        << ", \"length\": " << total_frames << "}\n";
  }

  // ── tasks.jsonl (overwrite — single task entry) ───────────────────────────
  {
    std::ofstream out(meta_dir + "/tasks.jsonl");
    out << "{\"task_index\": 0, \"task\": " << jsonStr(task_desc_) << "}\n";
  }

  // ── episodes_stats.jsonl (append — one line per episode) ──────────────────
  {
    auto writeStats = [&](std::ofstream & out, const std::string & name,
                          const FeatureStats & fs) {
      out << jsonStr(name) << ": {"
          << "\"max\": "  << floatVecToJson(fs.max_vals) << ", "
          << "\"mean\": " << floatVecToJson(fs.mean)     << ", "
          << "\"min\": "  << floatVecToJson(fs.min_vals) << ", "
          << "\"std\": "  << floatVecToJson(fs.std_dev)  << "}";
    };

    std::ofstream out(meta_dir + "/episodes_stats.jsonl", std::ios::app);
    out << "{\"episode_index\": " << episode_idx_ << ", \"stats\": {";
    writeStats(out, "observation.state", stats.obs_state);
    out << ", ";
    writeStats(out, "action", stats.action);
    out << ", ";
    writeStats(out, "timestamp", stats.timestamp);
    out << "}}\n";
  }

  // Update in-memory committed totals for subsequent episodes in this run.
  total_committed_episodes_ = new_total_episodes;
  total_committed_frames_   = new_total_frames;
}

bool SimRecorder::convertJsonlToParquet(const std::string & jsonl_path,
                                        const std::string & parquet_path) const
{
  // Find the bundled conversion script via ament_index.
  std::string script_path;
  try
  {
    script_path = ament_index_cpp::get_package_share_directory("mujoco_ros_utils")
                  + "/scripts/jsonl_to_lerobot_parquet.py";
  }
  catch (...)
  {
    script_path = "";
  }

  std::string cmd;
  if (!script_path.empty() && fs::exists(script_path))
    cmd = "python3 " + script_path + " " + jsonl_path + " " + parquet_path + " 2>&1";
  else
    cmd = "python3 -c \""
          "import json,pyarrow as pa,pyarrow.parquet as pq;"
          "rows=[json.loads(l) for l in open('" + jsonl_path + "') if l.strip()];"
          "pq.write_table(pa.Table.from_pylist(rows),'" + parquet_path + "')\" 2>&1";

  print_info("[SimRecorder] Running: %s\n", cmd.c_str());
  int ret = std::system(cmd.c_str());
  return ret == 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  HDF5
// ══════════════════════════════════════════════════════════════════════════════

hid_t SimRecorder::hdf5CreateJointDataset(hid_t group, const char * name, hsize_t n_joints)
{
  hsize_t init_dims[2]  = {0,              n_joints};
  hsize_t max_dims[2]   = {H5S_UNLIMITED,  n_joints};
  hsize_t chunk_dims[2] = {32,             n_joints};

  hid_t space = H5Screate_simple(2, init_dims, max_dims);
  hid_t dcpl  = H5Pcreate(H5P_DATASET_CREATE);
  H5Pset_chunk(dcpl, 2, chunk_dims);
  H5Pset_deflate(dcpl, 4);  // ZLIB level 4

  hid_t ds = H5Dcreate2(group, name, H5T_IEEE_F32LE, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
  H5Pclose(dcpl);
  H5Sclose(space);
  return ds;
}

hid_t SimRecorder::hdf5CreateImageDataset(const std::string & cam_ns, int h, int w)
{
  hsize_t init_dims[4]  = {0,             (hsize_t)h, (hsize_t)w, 3};
  hsize_t max_dims[4]   = {H5S_UNLIMITED, (hsize_t)h, (hsize_t)w, 3};
  hsize_t chunk_dims[4] = {1,             (hsize_t)h, (hsize_t)w, 3};  // one frame per chunk

  hid_t space = H5Screate_simple(4, init_dims, max_dims);
  hid_t dcpl  = H5Pcreate(H5P_DATASET_CREATE);
  H5Pset_chunk(dcpl, 4, chunk_dims);
  // Skip compression for images — they're already compressed-ish and the
  // per-frame fwrite cost of gzip on large frames adds up.

  // Build the dataset path: /observations/images/{cam_ns}
  const std::string ds_path = cam_ns;

  hid_t img_group = H5Gopen2(hdf5_file_id_, "/observations/images", H5P_DEFAULT);
  if (img_group < 0)
  {
    print_error("[SimRecorder] HDF5: failed to open /observations/images group\n");
    H5Pclose(dcpl);
    H5Sclose(space);
    return -1;
  }

  hid_t ds = H5Dcreate2(img_group, cam_ns.c_str(), H5T_STD_U8LE,
                         space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
  H5Gclose(img_group);
  H5Pclose(dcpl);
  H5Sclose(space);
  return ds;
}

void SimRecorder::hdf5AppendRow2D(hid_t ds, hsize_t row, hsize_t n_cols, const float * data)
{
  hsize_t new_size[2] = {row + 1, n_cols};
  H5Dset_extent(ds, new_size);

  hid_t fspace = H5Dget_space(ds);
  hsize_t start[2] = {row, 0};
  hsize_t count[2] = {1,   n_cols};
  H5Sselect_hyperslab(fspace, H5S_SELECT_SET, start, nullptr, count, nullptr);

  hid_t mspace = H5Screate_simple(2, count, nullptr);
  H5Dwrite(ds, H5T_IEEE_F32LE, mspace, fspace, H5P_DEFAULT, data);
  H5Sclose(mspace);
  H5Sclose(fspace);
}

void SimRecorder::hdf5AppendScalar(hid_t ds, hsize_t row, double value)
{
  hsize_t new_size[1] = {row + 1};
  H5Dset_extent(ds, new_size);

  hid_t fspace = H5Dget_space(ds);
  hsize_t start[1] = {row};
  hsize_t count[1] = {1};
  H5Sselect_hyperslab(fspace, H5S_SELECT_SET, start, nullptr, count, nullptr);

  hid_t mspace = H5Screate_simple(1, count, nullptr);
  H5Dwrite(ds, H5T_IEEE_F64LE, mspace, fspace, H5P_DEFAULT, &value);
  H5Sclose(mspace);
  H5Sclose(fspace);
}

void SimRecorder::hdf5AppendImage(hid_t ds, hsize_t frame_idx,
                                  int h, int w, const uint8_t * bgr_data)
{
  hsize_t new_size[4] = {frame_idx + 1, (hsize_t)h, (hsize_t)w, 3};
  H5Dset_extent(ds, new_size);

  hid_t fspace = H5Dget_space(ds);
  hsize_t start[4] = {frame_idx, 0, 0, 0};
  hsize_t count[4] = {1, (hsize_t)h, (hsize_t)w, 3};
  H5Sselect_hyperslab(fspace, H5S_SELECT_SET, start, nullptr, count, nullptr);

  hid_t mspace = H5Screate_simple(4, count, nullptr);
  H5Dwrite(ds, H5T_STD_U8LE, mspace, fspace, H5P_DEFAULT, bgr_data);
  H5Sclose(mspace);
  H5Sclose(fspace);
}

bool SimRecorder::openHDF5(const std::string & path)
{
  // Suppress HDF5 default error printing — we handle errors ourselves.
  H5Eset_auto2(H5E_DEFAULT, nullptr, nullptr);

  hdf5_file_id_ = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  if (hdf5_file_id_ < 0)
  {
    print_error("[SimRecorder] HDF5: failed to create file %s\n", path.c_str());
    return false;
  }

  const hsize_t n_joints = static_cast<hsize_t>(joint_names_.size());

  // Create group hierarchy
  hid_t g_obs  = H5Gcreate2(hdf5_file_id_, "/observations",        H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  hid_t g_imgs = H5Gcreate2(hdf5_file_id_, "/observations/images", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Gclose(g_imgs);

  // Joint datasets (2-D extensible float32)
  if (n_joints > 0)
  {
    hdf5_jpos_ds_ = hdf5CreateJointDataset(g_obs, "joint_positions",  n_joints);
    hdf5_jvel_ds_ = hdf5CreateJointDataset(g_obs, "joint_velocities", n_joints);
    hdf5_jeff_ds_ = hdf5CreateJointDataset(g_obs, "joint_efforts",    n_joints);
  }
  H5Gclose(g_obs);

  // Action dataset
  if (n_joints > 0)
    hdf5_act_ds_ = hdf5CreateJointDataset(hdf5_file_id_, "actions", n_joints);

  // Timestamp dataset (1-D extensible float64)
  {
    hsize_t init_dims[1]  = {0};
    hsize_t max_dims[1]   = {H5S_UNLIMITED};
    hsize_t chunk_dims[1] = {256};

    hid_t space = H5Screate_simple(1, init_dims, max_dims);
    hid_t dcpl  = H5Pcreate(H5P_DATASET_CREATE);
    H5Pset_chunk(dcpl, 1, chunk_dims);
    hdf5_ts_ds_ = H5Dcreate2(hdf5_file_id_, "timestamps",
                              H5T_IEEE_F64LE, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
    H5Pclose(dcpl);
    H5Sclose(space);
  }

  hdf5_n_rows_             = 0;
  hdf5_img_h_              = 0;
  hdf5_img_w_              = 0;
  hdf5_last_video_seq_.clear();
  hdf5_video_frame_count_ = 0;

  print_info("[SimRecorder] HDF5 file opened: %s\n", path.c_str());
  return true;
}

void SimRecorder::writeHDF5Frame(const SimSnapshot & snap)
{
  const hsize_t n_joints = static_cast<hsize_t>(joint_names_.size());

  // ── Camera images ──────────────────────────────────────────────────────────
  bool new_video_frame = false;

  for (const auto & ns : camera_namespaces_)
  {
    auto cit = snap.cameras.find(ns);
    if (cit == snap.cameras.end() || !cit->second.valid)
      continue;

    const auto & frame = cit->second;

    // Skip duplicate frames (same camera seq as last step).
    auto & last_seq = hdf5_last_video_seq_[ns];
    if (frame.seq == last_seq)
      continue;
    last_seq = frame.seq;

    // Create the image dataset lazily on the first frame.
    if (hdf5_img_ds_.find(ns) == hdf5_img_ds_.end())
    {
      hdf5_img_h_ = frame.height;
      hdf5_img_w_ = frame.width;
      hid_t img_ds = hdf5CreateImageDataset(ns, frame.height, frame.width);
      if (img_ds < 0)
        continue;
      hdf5_img_ds_[ns] = img_ds;
    }

    auto dit = hdf5_img_ds_.find(ns);
    if (dit == hdf5_img_ds_.end())
      continue;

    hdf5AppendImage(dit->second, static_cast<hsize_t>(hdf5_video_frame_count_),
                    frame.height, frame.width, frame.data.data());
    new_video_frame = true;
  }

  // Skip tabular accumulation if no new video frame (when cameras are present).
  if (!camera_namespaces_.empty() && !new_video_frame)
    return;

  // ── Joint state + action ───────────────────────────────────────────────────
  if (n_joints > 0)
  {
    std::vector<float> pos_row(n_joints, 0.0f);
    std::vector<float> vel_row(n_joints, 0.0f);
    std::vector<float> eff_row(n_joints, 0.0f);
    std::vector<float> act_row(n_joints, 0.0f);

    for (hsize_t j = 0; j < n_joints; ++j)
    {
      const auto & jname = joint_names_[j];
      auto jit = snap.joints.find(jname);
      if (jit != snap.joints.end())
      {
        const auto & js = jit->second;
        pos_row[j] = static_cast<float>(js.position);
        vel_row[j] = static_cast<float>(js.velocity);
        eff_row[j] = static_cast<float>(js.effort);
        double ctrl = pickPositionCtrl(js.actuator_ctrls, jname);
        act_row[j] = std::isfinite(ctrl) ? static_cast<float>(ctrl) : pos_row[j];
      }
    }

    hdf5AppendRow2D(hdf5_jpos_ds_, hdf5_n_rows_, n_joints, pos_row.data());
    hdf5AppendRow2D(hdf5_jvel_ds_, hdf5_n_rows_, n_joints, vel_row.data());
    hdf5AppendRow2D(hdf5_jeff_ds_, hdf5_n_rows_, n_joints, eff_row.data());
    hdf5AppendRow2D(hdf5_act_ds_,  hdf5_n_rows_, n_joints, act_row.data());
  }

  // ── Timestamp ─────────────────────────────────────────────────────────────
  const double rel_time = snap.sim_time - start_sim_time_;
  hdf5AppendScalar(hdf5_ts_ds_, hdf5_n_rows_, rel_time);

  ++hdf5_n_rows_;
  if (new_video_frame)
    ++hdf5_video_frame_count_;
}

bool SimRecorder::closeHDF5()
{
  if (hdf5_file_id_ < 0)
    return true;

  // Write root-level attributes
  auto writeStrAttr = [&](hid_t obj, const char * name, const std::string & value) {
    hid_t atype = H5Tcopy(H5T_C_S1);
    H5Tset_size(atype, value.size() + 1);
    H5Tset_strpad(atype, H5T_STR_NULLTERM);
    hid_t aspace = H5Screate(H5S_SCALAR);
    hid_t attr   = H5Acreate2(obj, name, atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr, atype, value.c_str());
    H5Aclose(attr);
    H5Sclose(aspace);
    H5Tclose(atype);
  };
  auto writeIntAttr = [&](hid_t obj, const char * name, int value) {
    hid_t aspace = H5Screate(H5S_SCALAR);
    hid_t attr   = H5Acreate2(obj, name, H5T_STD_I32LE, aspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr, H5T_NATIVE_INT, &value);
    H5Aclose(attr);
    H5Sclose(aspace);
  };
  auto writeF32Attr = [&](hid_t obj, const char * name, float value) {
    hid_t aspace = H5Screate(H5S_SCALAR);
    hid_t attr   = H5Acreate2(obj, name, H5T_IEEE_F32LE, aspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr, H5T_NATIVE_FLOAT, &value);
    H5Aclose(attr);
    H5Sclose(aspace);
  };

  writeStrAttr(hdf5_file_id_, "robot_name", aggregator_name_);
  writeStrAttr(hdf5_file_id_, "task",       task_desc_);
  writeIntAttr(hdf5_file_id_, "n_frames",   static_cast<int>(hdf5_n_rows_));
  writeF32Attr(hdf5_file_id_, "fps",        VIDEO_FPS);
  writeF32Attr(hdf5_file_id_, "duration",
               static_cast<float>(last_sim_time_ - start_sim_time_));

  // Write joint_names as a variable-length string array attribute
  if (!joint_names_.empty())
  {
    const size_t n = joint_names_.size();
    std::vector<const char *> cstrs(n);
    for (size_t i = 0; i < n; ++i)
      cstrs[i] = joint_names_[i].c_str();

    hid_t atype   = H5Tcopy(H5T_C_S1);
    H5Tset_size(atype, H5T_VARIABLE);
    hsize_t adims[1] = {n};
    hid_t aspace  = H5Screate_simple(1, adims, nullptr);
    hid_t attr    = H5Acreate2(hdf5_file_id_, "joint_names", atype, aspace,
                                H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr, atype, cstrs.data());
    H5Aclose(attr);
    H5Sclose(aspace);
    H5Tclose(atype);
  }

  // Close all datasets
  auto closeDs = [](hid_t & id) {
    if (id >= 0) { H5Dclose(id); id = -1; }
  };
  closeDs(hdf5_jpos_ds_);
  closeDs(hdf5_jvel_ds_);
  closeDs(hdf5_jeff_ds_);
  closeDs(hdf5_act_ds_);
  closeDs(hdf5_ts_ds_);
  for (auto & kv : hdf5_img_ds_)
    H5Dclose(kv.second);
  hdf5_img_ds_.clear();

  H5Fclose(hdf5_file_id_);
  hdf5_file_id_ = -1;

  print_confirm("[SimRecorder] HDF5 episode written: %s  (%zu frames)\n",
                hdf5_path_.c_str(), static_cast<size_t>(hdf5_n_rows_));
  return true;
}

void SimRecorder::discardHDF5()
{
  // Close without keeping the file.
  auto closeDs = [](hid_t & id) {
    if (id >= 0) { H5Dclose(id); id = -1; }
  };
  closeDs(hdf5_jpos_ds_);
  closeDs(hdf5_jvel_ds_);
  closeDs(hdf5_jeff_ds_);
  closeDs(hdf5_act_ds_);
  closeDs(hdf5_ts_ds_);
  for (auto & kv : hdf5_img_ds_)
    H5Dclose(kv.second);
  hdf5_img_ds_.clear();

  if (hdf5_file_id_ >= 0)
  {
    H5Fclose(hdf5_file_id_);
    hdf5_file_id_ = -1;
    if (fs::exists(hdf5_path_))
      fs::remove(hdf5_path_);
  }

  hdf5_n_rows_ = 0;
  hdf5_last_video_seq_.clear();
  hdf5_video_frame_count_ = 0;
  print_confirm("[SimRecorder] HDF5 episode discarded.\n");
}

}  // namespace MujocoRosUtils
