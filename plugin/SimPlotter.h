#pragma once

#include "mujoco_ros_utils/srv/plot_command.hpp"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace MujocoRosUtils
{

// ─── Data source ──────────────────────────────────────────────────────────────

enum class DataSourceType
{
  JointPos,    JointVel,    JointEff,    JointCtrl,
  BodyPosX,    BodyPosY,    BodyPosZ,
  BodyQuatW,   BodyQuatX,   BodyQuatY,   BodyQuatZ,
  BodyVelLinX, BodyVelLinY, BodyVelLinZ,
  BodyVelAngX, BodyVelAngY, BodyVelAngZ,
  Sensor,
  ActuatorCtrl, ActuatorForce,
  QposIdx, QvelIdx, QaccIdx, CtrlIdx,
  Unknown
};

struct DataSource
{
  DataSourceType type  = DataSourceType::Unknown;
  int            index  = -1;  ///< primary resolved index (into qpos/qvel/…)
  int            index2 = 0;   ///< secondary index (sensor component offset)
  std::string    name;         ///< object name for mj_name2id lookup
  std::string    raw;          ///< original string (for error messages)
  bool           valid  = false;
};

// ─── Ring buffer (fixed-capacity, wraps in-place) ────────────────────────────

struct RingBuffer
{
  std::vector<double> times;
  std::vector<double> values;
  int write_pos = 0;
  int count     = 0;
  int capacity;

  explicit RingBuffer(int cap) : times(cap, 0.0), values(cap, 0.0), capacity(cap) {}

  void push(double t, double v)
  {
    times[write_pos]  = t;
    values[write_pos] = v;
    write_pos = (write_pos + 1) % capacity;
    if (count < capacity) ++count;
  }

  int head() const { return (write_pos - count + capacity) % capacity; }

  /** Copy data in chronological order into out_t/out_v (resized to count). */
  void copyOrdered(std::vector<double> & out_t, std::vector<double> & out_v) const
  {
    out_t.resize(count);
    out_v.resize(count);
    const int h = head();
    for (int i = 0; i < count; ++i)
    {
      out_t[i] = times [(h + i) % capacity];
      out_v[i] = values[(h + i) % capacity];
    }
  }
};

// ─── Per-line configuration and data ─────────────────────────────────────────

struct LineConfig
{
  std::string label;
  DataSource  source;
  DataSource  source_x;         ///< X source for scatter plots (times[] stores X values)
  float       color[3] = {1.f, 1.f, 1.f};
  int         yaxis    = 1;   ///< 1 = left, 2 = right Y axis
  bool        paused   = false;
  bool        is_scatter = false; ///< true → PlotScatter; source_x drives the X axis
  bool        persist_on_reset = false; ///< true → ring buffer survives sim reset/reload
  RingBuffer  ring{500};
  /// Accumulated data from all past resets (only populated when persist_on_reset=true).
  /// On each reset the live ring is drained here and the ring is cleared for the new episode.
  std::vector<double> persist_xs;
  std::vector<double> persist_ys;
  uint64_t persist_gen   = 0;     ///< incremented each time persist_xs/ys change
  /// Cached axis bounds for the persist data (updated at reset, not every frame).
  double persist_xmin =  std::numeric_limits<double>::max();
  double persist_xmax = -std::numeric_limits<double>::max();
  double persist_ymin =  std::numeric_limits<double>::max();
  double persist_ymax = -std::numeric_limits<double>::max();
};

// ─── Per-plot configuration ───────────────────────────────────────────────────

struct PlotConfig
{
  std::string name;
  std::string title;

  /** Initial ImGui sub-window position (pixels from SDL2 window top-left). */
  float init_x = 10.f, init_y = 10.f, init_w = 480.f, init_h = 320.f;

  bool   auto_range_x  = true, auto_range_y  = true, auto_range_y2  = true;
  double range_x[2]    = {0, 0};
  double range_y[2]    = {0, 0};
  double range_y2[2]   = {0, 0};

  /** Scrolling time window width in seconds for line plots.
   *  When auto-scroll is active the x-axis always shows
   *  [latest_t - time_window, latest_t].  Set to 0 to show all data. */
  double time_window = 10.0;

  bool paused  = false;
  bool has_y2  = false;
  bool persist_on_reset = false; ///< true → all lines inherit persist unless overridden
  /** Max points passed to ImPlot per scatter series per frame.
   *  Points outside the visible axis range are discarded first, then the
   *  remainder is stride-sampled down to this limit.  Keeps cursor/tooltip
   *  search fast regardless of how much history has accumulated. */
  int scatter_render_cap = 3000;
  /** Minimum pixel separation between rendered scatter points (grid deduplication).
   *  A 2-D grid of (plot_w/res)×(plot_h/res) cells is built per render call;
   *  only the first point that maps to each cell is drawn.  Larger values = fewer
   *  points and faster cursor search.  0 = disabled (only stride cap applies). */
  int scatter_pixel_res = 4;

  std::vector<LineConfig> lines;
};

// ─── SimPlotter MuJoCo passive plugin ────────────────────────────────────────

/**
 * SimPlotter — real-time in-process signal plotter for MuJoCo simulations.
 *
 * Renders a floating ImPlot window (GPU-accelerated) in a separate SDL2/OpenGL
 * thread.  No ROS DDS traffic; reads all data directly from mjData.
 *
 * XML instantiation
 * -----------------
 *   <extension>
 *     <plugin plugin="MujocoRosUtils::SimPlotter">
 *       <instance name="cobot_plotter">
 *         <config key="window_w"           value="1280"/>
 *         <config key="window_h"           value="800"/>
 *         <config key="update_every"       value="10"/>
 *         <config key="num_plots"          value="1"/>
 *         <config key="plot0_title"        value="Joint Positions"/>
 *         <config key="plot0_pos"          value="tl 10 10 600 300"/>
 *         <config key="plot0_range_x"      value="auto"/>
 *         <config key="plot0_range_y"      value="auto"/>
 *         <config key="plot0_max_pts"      value="500"/>
 *         <config key="plot0_num_lines"    value="2"/>
 *         <config key="plot0_line0_label"  value="shoulder_pan"/>
 *         <config key="plot0_line0_src"    value="joint.shoulder_pan_joint.position"/>
 *         <config key="plot0_line0_color"  value="1.0 0.3 0.3"/>
 *         <config key="plot0_line1_label"  value="elbow"/>
 *         <config key="plot0_line1_src"    value="joint.elbow_joint.velocity"/>
 *         <config key="plot0_line1_color"  value="0.3 1.0 0.3"/>
 *         <config key="plot0_line1_yaxis"  value="2"/>
 *       </instance>
 *     </plugin>
 *   </extension>
 *
 * Data source syntax
 * ------------------
 *   joint.<name>.position | velocity | effort | ctrl
 *   body.<name>.pos.x | y | z
 *   body.<name>.quat.w | x | y | z
 *   body.<name>.vel_lin.x | y | z
 *   body.<name>.vel_ang.x | y | z
 *   sensor.<name>           (scalar sensor; use sensor.<name>.<N> for Nth component)
 *   actuator.<name>.ctrl | force
 *   qpos.<index> | qvel.<index> | qacc.<index> | ctrl.<index>
 *
 * Persist on reset
 * ----------------
 *   Add persist=true to a plot or line config to retain ring-buffer data across
 *   simulation resets (Ctrl+R) and episode boundaries.  Useful for scatter plots
 *   where you want to accumulate the torque-velocity manifold over many runs.
 *     plot level:  "title=...;persist=true"          → all lines in that plot persist
 *     line level:  "label=...;src=...;persist=true"  → only this line persists
 *   Line-level setting takes priority over plot-level.
 *
 * ROS 2 service
 * -------------
 *   <instance_name>/plot_command  [mujoco_ros_utils/srv/PlotCommand]
 *   Supports: add_plot, del_plot, add_line, del_line, mod_plot, mod_line,
 *             clear, pause, resume, export, list
 */
class SimPlotter
{
public:
  static constexpr char kName[] = "MujocoRosUtils::SimPlotter";

  static void         RegisterPlugin();
  static SimPlotter * Create(const mjModel * m, mjData * d, int plugin_id);

  SimPlotter(SimPlotter &&)            = delete;
  SimPlotter(const SimPlotter &)       = delete;
  ~SimPlotter();

  void reset  (const mjModel * m, int plugin_id);
  void compute(const mjModel * m, mjData * d, int plugin_id);

private:
  SimPlotter(std::vector<PlotConfig> plots,
             int update_every, int win_w, int win_h,
             std::string node_name,
             const mjModel * m);

  // ── Data source helpers ───────────────────────────────────────────────────
  static DataSource parseSource(const std::string & src);
  static void       resolveSource(DataSource & ds, const mjModel * m);
  static double     readSource(const DataSource & ds, const mjData * d);

  // ── Process / thread management ───────────────────────────────────────────
  void startRenderThread();
  void startRosThread(const std::string & node_name);
  void stopThreads();
  void renderLoop();  ///< runs in render_thread_

  // ── Service helpers ───────────────────────────────────────────────────────
  using PlotCmd     = mujoco_ros_utils::srv::PlotCommand;
  using PlotCmdReq  = PlotCmd::Request::ConstSharedPtr;
  using PlotCmdRes  = PlotCmd::Response::SharedPtr;

  void handlePlotCommand(PlotCmdReq req, PlotCmdRes res);

  static std::map<std::string, std::string> parseArgs(const std::vector<std::string> & args);
  void applyPlotFromArgs (PlotConfig & pc, const std::map<std::string, std::string> & kv);
  void applyLineFromArgs (LineConfig & lc, const std::map<std::string, std::string> & kv);
  static void parsePos   (const std::string & pos_str,
                          float win_w, float win_h,
                          float & out_x, float & out_y,
                          float & out_w, float & out_h);

  // ── State ─────────────────────────────────────────────────────────────────
  mutable std::mutex      mutex_;
  std::vector<PlotConfig> plots_;
  int                     update_every_ = 10;
  int                     step_         = 0;
  double                  last_compute_time_ = -1.0;  ///< detects sim pause (time not advancing)
  int                     win_w_        = 1280;
  int                     win_h_        = 800;
  const mjModel *         model_        = nullptr;  ///< saved for service-time resolution

  std::atomic<bool>  running_{false};
  // Shared-ownership running flag: lets renderLoop() safely read it even after
  // the plugin object has been destroyed (which can happen when the old plugin
  // is destroyed just after the new one is created on CTRL+L reload).
  std::shared_ptr<std::atomic<bool>> running_sptr_;
  std::thread        render_thread_;
  std::thread        ros_thread_;

  rclcpp::Node::SharedPtr                                    node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::atomic<bool>                                          stop_executor_{false};
  rclcpp::Service<PlotCmd>::SharedPtr                        cmd_srv_;
};

}  // namespace MujocoRosUtils
