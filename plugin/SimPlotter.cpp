#include "SimPlotter.h"
#include "mujoco_utils.hpp"

#include <mujoco/mujoco.h>

// ── ImGui / ImPlot ────────────────────────────────────────────────────────────
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdlrenderer2.h"
#include "implot.h"

// ── SDL2 ──────────────────────────────────────────────────────────────────────
// Best path  : SDL2 + EGL-backed OpenGL 3.2 (GPU, no GLX).
//              SDL_VIDEO_X11_FORCE_EGL=1 routes SDL2 through libEGL_nvidia (GLVND)
//              instead of libGLX_nvidia.  EGL and GLX are independent code paths in
//              the NVIDIA driver — no shared state, no X11 protocol conflict with
//              GLFW's GLX context, no physics stalls.
// Fallback   : SDL2 software renderer (CPU, XPutImage/XShmPutImage).
//              Always works; zero GL/EGL dependency.
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
// ImGui's embedded glad loader — GL types + function pointers via
// SDL_GL_GetProcAddress → eglGetProcAddress.  No libGL/libGLX linkage required.
#include "imgui_impl_opengl3_loader.h"
#include <dlfcn.h>

// ── Debug: X11 error interception + backtrace ─────────────────────────────────
#include <X11/Xlib.h>
#include <execinfo.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace MujocoRosUtils
{

// ─── Debug: X11 error handler that prints backtrace instead of crashing ───────
// Installed in the SimPlotter constructor so we can see EXACTLY what triggers
// the GLX BadValue error and from which call stack.

// ─── Global render-slot (one active render thread at a time) ─────────────────
// MuJoCo creates the NEW plugin instance BEFORE destroying the OLD one on every
// Ctrl+L reload.  That leaves two render threads alive simultaneously, each
// calling ImGui/ImPlot functions that mutate global state → corruption/segfault.
// Solution: a "slot" that only one render thread may occupy.  A new thread
// blocks on the condition variable until the previous thread has fully cleaned
// up and released the slot.  A SDL refcount ensures SDL_Quit only fires after
// the last owner exits.
static std::mutex              g_render_slot_mtx;
static std::condition_variable g_render_slot_cv;
static bool                    g_render_slot_taken = false;
static int                     g_sdl_refcount      = 0;

static int (*g_prev_x11_handler)(Display *, XErrorEvent *) = nullptr;

static int debugX11ErrorHandler(Display * dpy, XErrorEvent * ev)
{
  char errtxt[256] = {};
  XGetErrorText(dpy, ev->error_code, errtxt, sizeof(errtxt));

  fprintf(stderr,
    "\n[SimPlotter DEBUG] *** X11 Error ***\n"
    "  error_code   = %d (%s)\n"
    "  request_code = %d (152=GLX)\n"
    "  minor_code   = %d (3=X_GLXCreateContext)\n"
    "  serial       = %lu\n"
    "  resourceid   = 0x%lx\n"
    "  Backtrace:\n",
    (int)ev->error_code, errtxt,
    (int)ev->request_code,
    (int)ev->minor_code,
    ev->serial,
    ev->resourceid);

  void * bt[64];
  int n = backtrace(bt, 64);
  backtrace_symbols_fd(bt, n, STDERR_FILENO);
  fprintf(stderr, "[SimPlotter DEBUG] *** end backtrace ***\n\n");
  fflush(stderr);

  // Suppress GLX probe errors so GLFW's fatal handler doesn't kill the process.
  // For all other errors, forward to the previous handler.
  if (ev->request_code == 152)  // GLX major opcode
    return 0;
  if (g_prev_x11_handler)
    return g_prev_x11_handler(dpy, ev);
  return 0;
}



constexpr char ATTR_WIN_W[]        = "window_w";
constexpr char ATTR_WIN_H[]        = "window_h";
constexpr char ATTR_UPDATE_EVERY[] = "update_every";
constexpr char ATTR_NUM_PLOTS[]    = "num_plots";

// ─── Auto-color palette (matplotlib tab10) ────────────────────────────────────

static void autoColor(int idx, float (& out)[3])
{
  static const float kPalette[][3] = {
    {0.12f, 0.47f, 0.71f}, {1.00f, 0.50f, 0.05f}, {0.17f, 0.63f, 0.17f},
    {0.84f, 0.15f, 0.16f}, {0.58f, 0.40f, 0.74f}, {0.55f, 0.34f, 0.29f},
    {0.89f, 0.47f, 0.76f}, {0.50f, 0.50f, 0.50f}, {0.74f, 0.74f, 0.13f},
    {0.09f, 0.75f, 0.81f},
  };
  const int n = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
  out[0] = kPalette[idx % n][0];
  out[1] = kPalette[idx % n][1];
  out[2] = kPalette[idx % n][2];
}

// ─── Data source parsing ──────────────────────────────────────────────────────

DataSource SimPlotter::parseSource(const std::string & src)
{
  DataSource ds;
  ds.raw = src;

  std::vector<std::string> parts;
  std::istringstream       ss(src);
  std::string              tok;
  while (std::getline(ss, tok, '.'))
    if (!tok.empty()) parts.push_back(tok);

  if (parts.size() < 2) return ds;

  const auto & cat = parts[0];

  // ── qpos / qvel / qacc / ctrl indexed — and sensor.<name> ───────────────
  if (parts.size() == 2)
  {
    // sensor.<name>  (scalar sensor, component 0)
    if (cat == "sensor")
    {
      ds.name = parts[1]; ds.type = DataSourceType::Sensor; ds.index2 = 0;
      return ds;
    }
    auto maybeIdx = [&](DataSourceType t) {
      try { ds.type = t; ds.index = std::stoi(parts[1]); ds.valid = true; }
      catch (...) {}
    };
    if      (cat == "qpos") maybeIdx(DataSourceType::QposIdx);
    else if (cat == "qvel") maybeIdx(DataSourceType::QvelIdx);
    else if (cat == "qacc") maybeIdx(DataSourceType::QaccIdx);
    else if (cat == "ctrl") maybeIdx(DataSourceType::CtrlIdx);
    return ds;
  }

  if (parts.size() < 3) return ds;

  const auto & name  = parts[1];
  const auto & field = parts[2];

  // ── joint.<name>.<field> ──────────────────────────────────────────────────
  if (cat == "joint")
  {
    ds.name = name;
    if      (field == "position") ds.type = DataSourceType::JointPos;
    else if (field == "velocity") ds.type = DataSourceType::JointVel;
    else if (field == "effort")   ds.type = DataSourceType::JointEff;
    else if (field == "ctrl")     ds.type = DataSourceType::JointCtrl;
    else return ds;
    // index resolved later by resolveSource
    return ds;
  }

  // ── actuator.<name>.<field> ───────────────────────────────────────────────
  if (cat == "actuator")
  {
    ds.name = name;
    if      (field == "ctrl")  ds.type = DataSourceType::ActuatorCtrl;
    else if (field == "force") ds.type = DataSourceType::ActuatorForce;
    else return ds;
    return ds;
  }

  // ── sensor.<name>.<component> — field IS the component index ─────────────
  if (cat == "sensor")
  {
    ds.name   = name;
    ds.type   = DataSourceType::Sensor;
    try { ds.index2 = std::stoi(field); } catch (...) { ds.index2 = 0; }
    return ds;
  }

  // ── body.<name>.<group>.<axis> ────────────────────────────────────────────
  if (cat == "body" && parts.size() >= 4)
  {
    ds.name = name;
    const auto & grp  = parts[2];
    const auto & axis = parts[3];
    auto axisIdx = [&](int x, int y, int z) {
      if (axis == "x") return x;
      if (axis == "y") return y;
      if (axis == "z") return z;
      return -1;
    };
    if (grp == "pos")
    {
      int a = axisIdx(0, 1, 2);
      if (a < 0) return ds;
      ds.type  = static_cast<DataSourceType>(
          static_cast<int>(DataSourceType::BodyPosX) + a);
      return ds;
    }
    if (grp == "quat" && parts.size() >= 5)
    {
      const auto & comp = parts[4];
      int c = (comp == "w") ? 0 : (comp == "x") ? 1 : (comp == "y") ? 2 : (comp == "z") ? 3 : -1;
      if (c < 0) return ds;
      ds.type = static_cast<DataSourceType>(
          static_cast<int>(DataSourceType::BodyQuatW) + c);
      return ds;
    }
    if (grp == "vel_lin")
    {
      int a = axisIdx(0, 1, 2);
      if (a < 0) return ds;
      ds.type = static_cast<DataSourceType>(
          static_cast<int>(DataSourceType::BodyVelLinX) + a);
      return ds;
    }
    if (grp == "vel_ang")
    {
      int a = axisIdx(0, 1, 2);
      if (a < 0) return ds;
      ds.type = static_cast<DataSourceType>(
          static_cast<int>(DataSourceType::BodyVelAngX) + a);
      return ds;
    }
  }

  return ds;
}

void SimPlotter::resolveSource(DataSource & ds, const mjModel * m)
{
  if (ds.type == DataSourceType::Unknown) return;

  // Index-based sources need only bounds checking
  if (ds.type == DataSourceType::QposIdx)
  {
    if (ds.index >= 0 && ds.index < m->nq) ds.valid = true;
    else mju_warning("[SimPlotter] qpos index %d out of range (nq=%d)", ds.index, (int)m->nq);
    return;
  }
  if (ds.type == DataSourceType::QvelIdx)
  {
    if (ds.index >= 0 && ds.index < m->nv) ds.valid = true;
    else mju_warning("[SimPlotter] qvel index %d out of range", ds.index);
    return;
  }
  if (ds.type == DataSourceType::QaccIdx)
  {
    if (ds.index >= 0 && ds.index < m->nv) ds.valid = true;
    else mju_warning("[SimPlotter] qacc index %d out of range", ds.index);
    return;
  }
  if (ds.type == DataSourceType::CtrlIdx)
  {
    if (ds.index >= 0 && ds.index < m->nu) ds.valid = true;
    else mju_warning("[SimPlotter] ctrl index %d out of range", ds.index);
    return;
  }

  // Name-based sources
  if (ds.type == DataSourceType::JointPos  ||
      ds.type == DataSourceType::JointVel  ||
      ds.type == DataSourceType::JointEff  ||
      ds.type == DataSourceType::JointCtrl)
  {
    int jid = mj_name2id(m, mjOBJ_JOINT, ds.name.c_str());
    if (jid < 0)
    {
      mju_warning("[SimPlotter] joint '%s' not found in model", ds.name.c_str());
      return;
    }
    if (ds.type == DataSourceType::JointPos)
    {
      ds.index = m->jnt_qposadr[jid];
    }
    else if (ds.type == DataSourceType::JointVel || ds.type == DataSourceType::JointEff)
    {
      ds.index = m->jnt_dofadr[jid];
    }
    else  // JointCtrl: find first actuator that drives this joint
    {
      ds.index = -1;
      for (int i = 0; i < m->nu; ++i)
      {
        if (m->actuator_trntype[i] == mjTRN_JOINT && m->actuator_trnid[i * 2] == jid)
        {
          ds.index = i;
          break;
        }
      }
      if (ds.index < 0)
      {
        mju_warning("[SimPlotter] no actuator found for joint '%s'", ds.name.c_str());
        return;
      }
    }
    ds.valid = true;
    return;
  }

  if (ds.type == DataSourceType::ActuatorCtrl ||
      ds.type == DataSourceType::ActuatorForce)
  {
    int aid = mj_name2id(m, mjOBJ_ACTUATOR, ds.name.c_str());
    if (aid < 0)
    {
      mju_warning("[SimPlotter] actuator '%s' not found", ds.name.c_str());
      return;
    }
    ds.index = aid;
    ds.valid = true;
    return;
  }

  if (ds.type == DataSourceType::Sensor)
  {
    int sid = mj_name2id(m, mjOBJ_SENSOR, ds.name.c_str());
    if (sid < 0)
    {
      mju_warning("[SimPlotter] sensor '%s' not found", ds.name.c_str());
      return;
    }
    ds.index = m->sensor_adr[sid];  // base address in sensordata
    // ds.index2 is the component offset (set during parse)
    if (ds.index + ds.index2 >= m->nsensordata)
    {
      mju_warning("[SimPlotter] sensor '%s' component %d out of range", ds.name.c_str(), ds.index2);
      return;
    }
    ds.valid = true;
    return;
  }

  // Body sources
  if (ds.type >= DataSourceType::BodyPosX && ds.type <= DataSourceType::BodyVelAngZ)
  {
    int bid = mj_name2id(m, mjOBJ_BODY, ds.name.c_str());
    if (bid < 0)
    {
      mju_warning("[SimPlotter] body '%s' not found", ds.name.c_str());
      return;
    }
    ds.index = bid;
    ds.valid = true;
    return;
  }
}

double SimPlotter::readSource(const DataSource & ds, const mjData * d)
{
  if (!ds.valid) return 0.0;
  switch (ds.type)
  {
    case DataSourceType::JointPos:     return d->qpos[ds.index];
    case DataSourceType::JointVel:     return d->qvel[ds.index];
    case DataSourceType::JointEff:     return d->qfrc_actuator[ds.index];
    case DataSourceType::JointCtrl:    return d->ctrl[ds.index];
    case DataSourceType::BodyPosX:     return d->xpos[ds.index * 3 + 0];
    case DataSourceType::BodyPosY:     return d->xpos[ds.index * 3 + 1];
    case DataSourceType::BodyPosZ:     return d->xpos[ds.index * 3 + 2];
    case DataSourceType::BodyQuatW:    return d->xquat[ds.index * 4 + 0];
    case DataSourceType::BodyQuatX:    return d->xquat[ds.index * 4 + 1];
    case DataSourceType::BodyQuatY:    return d->xquat[ds.index * 4 + 2];
    case DataSourceType::BodyQuatZ:    return d->xquat[ds.index * 4 + 3];
    case DataSourceType::BodyVelLinX:  return d->cvel[ds.index * 6 + 3];
    case DataSourceType::BodyVelLinY:  return d->cvel[ds.index * 6 + 4];
    case DataSourceType::BodyVelLinZ:  return d->cvel[ds.index * 6 + 5];
    case DataSourceType::BodyVelAngX:  return d->cvel[ds.index * 6 + 0];
    case DataSourceType::BodyVelAngY:  return d->cvel[ds.index * 6 + 1];
    case DataSourceType::BodyVelAngZ:  return d->cvel[ds.index * 6 + 2];
    case DataSourceType::Sensor:       return d->sensordata[ds.index + ds.index2];
    case DataSourceType::ActuatorCtrl: return d->ctrl[ds.index];
    case DataSourceType::ActuatorForce: return d->actuator_force[ds.index];
    case DataSourceType::QposIdx:      return d->qpos[ds.index];
    case DataSourceType::QvelIdx:      return d->qvel[ds.index];
    case DataSourceType::QaccIdx:      return d->qacc[ds.index];
    case DataSourceType::CtrlIdx:      return d->ctrl[ds.index];
    default:                           return 0.0;
  }
}

// ─── Config helpers ───────────────────────────────────────────────────────────

/** Parse "anchor off_x off_y w h" into ImGui window pos/size. */
void SimPlotter::parsePos(const std::string & pos_str,
                          float win_w, float win_h,
                          float & out_x, float & out_y,
                          float & out_w, float & out_h)
{
  char   anchor[8] = "tl";
  float  ox = 10.f, oy = 10.f, pw = 480.f, ph = 320.f;
  std::sscanf(pos_str.c_str(), "%7s %f %f %f %f", anchor, &ox, &oy, &pw, &ph);

  std::string a(anchor);
  if      (a == "tl") { out_x = ox;                        out_y = oy; }
  else if (a == "tc") { out_x = (win_w - pw) * 0.5f + ox;  out_y = oy; }
  else if (a == "tr") { out_x = win_w - pw - ox;            out_y = oy; }
  else if (a == "bl") { out_x = ox;                        out_y = win_h - ph - oy; }
  else if (a == "br") { out_x = win_w - pw - ox;            out_y = win_h - ph - oy; }
  else                { out_x = ox;                        out_y = oy; }  // default tl

  out_w = pw;
  out_h = ph;
}

std::map<std::string, std::string>
SimPlotter::parseArgs(const std::vector<std::string> & args)
{
  std::map<std::string, std::string> kv;
  for (const auto & a : args)
  {
    const auto eq = a.find('=');
    if (eq == std::string::npos) continue;
    kv[a.substr(0, eq)] = a.substr(eq + 1);
  }
  return kv;
}

void SimPlotter::applyPlotFromArgs(PlotConfig & pc,
                                   const std::map<std::string, std::string> & kv)
{
  if (kv.count("pos"))
    parsePos(kv.at("pos"), static_cast<float>(win_w_), static_cast<float>(win_h_),
             pc.init_x, pc.init_y, pc.init_w, pc.init_h);

  if (kv.count("range_x"))
  {
    const auto & v = kv.at("range_x");
    if (v == "auto") { pc.auto_range_x = true; }
    else { std::sscanf(v.c_str(), "%lf %lf", &pc.range_x[0], &pc.range_x[1]); pc.auto_range_x = false; }
  }
  if (kv.count("range_y"))
  {
    const auto & v = kv.at("range_y");
    if (v == "auto") { pc.auto_range_y = true; }
    else { std::sscanf(v.c_str(), "%lf %lf", &pc.range_y[0], &pc.range_y[1]); pc.auto_range_y = false; }
  }
  if (kv.count("range_y2"))
  {
    const auto & v = kv.at("range_y2");
    if (v == "auto") { pc.auto_range_y2 = true; pc.has_y2 = true; }
    else { std::sscanf(v.c_str(), "%lf %lf", &pc.range_y2[0], &pc.range_y2[1]);
           pc.auto_range_y2 = false; pc.has_y2 = true; }
  }
  if (kv.count("max_pts"))
  {
    const int n = std::stoi(kv.at("max_pts"));
    for (auto & l : pc.lines)
      if (l.ring.capacity != n)
        l.ring = RingBuffer(n);  // reset ring buffer with new capacity
  }
  if (kv.count("update_every")) { /* handled at plugin level, not per-plot */ }
  if (kv.count("title")) pc.title = kv.at("title");
}

void SimPlotter::applyLineFromArgs(LineConfig & lc,
                                   const std::map<std::string, std::string> & kv)
{
  if (kv.count("src"))
  {
    lc.source = parseSource(kv.at("src"));
    if (model_) resolveSource(lc.source, model_);
  }
  if (kv.count("color"))
    std::sscanf(kv.at("color").c_str(), "%f %f %f", &lc.color[0], &lc.color[1], &lc.color[2]);
  if (kv.count("yaxis"))
  {
    lc.yaxis = std::stoi(kv.at("yaxis"));
  }
}

// ─── RegisterPlugin ───────────────────────────────────────────────────────────

void SimPlotter::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name           = kName;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  // Declare every key that can appear in a <config> element.
  // MuJoCo errors on any undeclared key, so we pre-generate all slot names:
  //   4 global + plot{N} + plot{N}_line{L}  (N=0..7, L=0..7) = 76 total
  static std::vector<std::string> s_attr_names;
  static std::vector<const char *> s_attr_ptrs;
  if (s_attr_names.empty())
  {
    s_attr_names = { ATTR_WIN_W, ATTR_WIN_H, ATTR_UPDATE_EVERY, ATTR_NUM_PLOTS };
    char buf[32];
    for (int p = 0; p < 8; ++p)
    {
      std::snprintf(buf, sizeof(buf), "plot%d", p);
      s_attr_names.emplace_back(buf);
      for (int l = 0; l < 8; ++l)
      {
        std::snprintf(buf, sizeof(buf), "plot%d_line%d", p, l);
        s_attr_names.emplace_back(buf);
      }
    }
    s_attr_ptrs.reserve(s_attr_names.size());
    for (const auto & n : s_attr_names)
      s_attr_ptrs.push_back(n.c_str());
  }
  plugin.nattribute = static_cast<int>(s_attr_ptrs.size());
  plugin.attributes = s_attr_ptrs.data();

  plugin.nstate = +[](const mjModel *, int) { return 0; };

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) -> int {
    auto * inst = SimPlotter::Create(m, d, plugin_id);
    if (!inst) return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id) {
    delete reinterpret_cast<SimPlotter *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id) {
    reinterpret_cast<SimPlotter *>(plugin_data)->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int) {
    reinterpret_cast<SimPlotter *>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
  print_confirm("Successfully registered 'MujocoRosUtils::SimPlotter' plugin\n");
}

// ─── Create ───────────────────────────────────────────────────────────────────

SimPlotter * SimPlotter::Create(const mjModel * m, mjData *, int plugin_id)
{
  auto cfg = [&](const char * key) -> std::string {
    const char * v = mj_getPluginConfig(m, plugin_id, key);
    return (v && v[0]) ? std::string(v) : std::string{};
  };
  auto cfgI = [&](const char * key, int def) -> int {
    const char * v = mj_getPluginConfig(m, plugin_id, key);
    return (v && v[0]) ? std::stoi(v) : def;
  };
  auto mk = [](const char * fmt, auto... args) -> std::string {
    char buf[128];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    return buf;
  };

  const int win_w        = cfgI(ATTR_WIN_W,        1280);
  const int win_h        = cfgI(ATTR_WIN_H,         800);
  const int update_every = cfgI(ATTR_UPDATE_EVERY,   10);
  const int num_plots    = cfgI(ATTR_NUM_PLOTS,        0);

  // Retrieve instance name (used for ROS 2 node name)
  const char * inst_name_c = mj_id2name(m, mjOBJ_PLUGIN, plugin_id);
  std::string  node_name   = inst_name_c ? std::string(inst_name_c) : "sim_plotter";

  std::vector<PlotConfig> plots;

  // Parse compact attribute format.
  // Each plot uses two declared attribute slots:
  //   "plot{N}"         → semicolon-separated key=value plot config
  //   "plot{N}_line{L}" → semicolon-separated key=value per-line config
  //                       (iterate L=0..7 until the value is empty)
  // Example:
  //   plot0       = "title=Kinematics;pos=tl 0 0 700 320;max_pts=1500"
  //   plot0_line0 = "label=velocity;src=joint.arm.velocity;color=1 0.5 0;y=1"

  auto parsePacked = [](const std::string & packed) -> std::map<std::string, std::string>
  {
    std::map<std::string, std::string> m;
    std::istringstream ss(packed);
    std::string token;
    while (std::getline(ss, token, ';'))
    {
      if (token.empty()) continue;
      auto eq = token.find('=');
      if (eq == std::string::npos) continue;
      m[token.substr(0, eq)] = token.substr(eq + 1);
    }
    return m;
  };

  int total_line_idx = 0;
  for (int p = 0; p < num_plots; ++p)
  {
    PlotConfig pc;
    pc.name = mk("plot%d", p);

    // ── Plot-level config ──────────────────────────────────────────────────
    const auto pmap = parsePacked(cfg(pc.name.c_str()));

    pc.title = pmap.count("title") ? pmap.at("title") : pc.name;

    if (pmap.count("pos"))
      parsePos(pmap.at("pos"), static_cast<float>(win_w), static_cast<float>(win_h),
               pc.init_x, pc.init_y, pc.init_w, pc.init_h);
    else
    {
      pc.init_x = 10.f + static_cast<float>(p) * 10.f;
      pc.init_y = 10.f + static_cast<float>(p) * 10.f;
    }

    auto parseRange = [&](const char * key, bool & autoF, double (&rng)[2])
    {
      if (!pmap.count(key) || pmap.at(key) == "auto") { autoF = true; return; }
      autoF = false;
      std::sscanf(pmap.at(key).c_str(), "%lf %lf", &rng[0], &rng[1]);
    };
    parseRange("range_x",  pc.auto_range_x,  pc.range_x);
    parseRange("range_y",  pc.auto_range_y,  pc.range_y);
    parseRange("range_y2", pc.auto_range_y2, pc.range_y2);
    if (!pc.auto_range_y2) pc.has_y2 = true;

    const int max_pts = pmap.count("max_pts") ? std::stoi(pmap.at("max_pts")) : 500;

    // ── Per-line config ────────────────────────────────────────────────────
    for (int l = 0; l < 8; ++l)
    {
      const std::string line_val = cfg(mk("plot%d_line%d", p, l).c_str());
      if (line_val.empty()) break;

      const auto lmap = parsePacked(line_val);

      // src / src_y = Y source; src_x = X source (scatter only)
      const std::string y_src_str =
          lmap.count("src_y") ? lmap.at("src_y") :
          lmap.count("src")   ? lmap.at("src")   : "";
      if (y_src_str.empty())
      {
        mju_warning("[SimPlotter] plot%d_line%d missing 'src' — skipping", p, l);
        continue;
      }

      LineConfig lc;
      lc.ring       = RingBuffer(max_pts);
      lc.source     = parseSource(y_src_str);
      resolveSource(lc.source, m);

      // Scatter: activated by type=scatter or presence of src_x
      const bool has_src_x = lmap.count("src_x") && !lmap.at("src_x").empty();
      lc.is_scatter = (lmap.count("type") && lmap.at("type") == "scatter") || has_src_x;
      if (has_src_x)
      {
        lc.source_x = parseSource(lmap.at("src_x"));
        resolveSource(lc.source_x, m);
        if (!lc.source_x.valid)
          mju_warning("[SimPlotter] plot%d_line%d: src_x '%s' could not be resolved",
                      p, l, lmap.at("src_x").c_str());
      }

      lc.label = lmap.count("label") ? lmap.at("label") : y_src_str;

      if (lmap.count("color"))
        std::sscanf(lmap.at("color").c_str(), "%f %f %f", &lc.color[0], &lc.color[1], &lc.color[2]);
      else
        autoColor(total_line_idx, lc.color);

      // accept both "y" and "yaxis" as the Y-axis key
      if      (lmap.count("y"))     lc.yaxis = std::stoi(lmap.at("y"));
      else if (lmap.count("yaxis")) lc.yaxis = std::stoi(lmap.at("yaxis"));
      if (lc.yaxis == 2) pc.has_y2 = true;

      pc.lines.push_back(std::move(lc));
      ++total_line_idx;
    }

    plots.push_back(std::move(pc));
  }

  print_confirm("[SimPlotter] Creating instance '%s': %d plots, win %dx%d\n",
                node_name.c_str(), num_plots, win_w, win_h);

  return new SimPlotter(std::move(plots), update_every, win_w, win_h, node_name, m);
}

// ─── Constructor / Destructor ─────────────────────────────────────────────────

SimPlotter::SimPlotter(std::vector<PlotConfig> plots,
                       int update_every, int win_w, int win_h,
                       std::string node_name,
                       const mjModel * m)
    : plots_(std::move(plots))
    , update_every_(update_every)
    , win_w_(win_w)
    , win_h_(win_h)
    , model_(m)
{
  // Install debug X11 error handler FIRST — catches whatever triggers serial 115.
  g_prev_x11_handler = XSetErrorHandler(debugX11ErrorHandler);
  fprintf(stderr, "[SimPlotter DEBUG] X11 error handler installed\n"); fflush(stderr);

  fprintf(stderr, "[SimPlotter DEBUG] calling startRenderThread()\n"); fflush(stderr);
  startRenderThread();
  fprintf(stderr, "[SimPlotter DEBUG] startRenderThread() returned\n"); fflush(stderr);

  fprintf(stderr, "[SimPlotter DEBUG] calling startRosThread()\n"); fflush(stderr);
  startRosThread(node_name);
  fprintf(stderr, "[SimPlotter DEBUG] startRosThread() returned\n"); fflush(stderr);
}

SimPlotter::~SimPlotter()
{
  stopThreads();
}

// ─── reset / compute ──────────────────────────────────────────────────────────

void SimPlotter::reset(const mjModel *, int)
{
  // Clear all ring buffers — render thread sees empty data on next snapshot
  std::lock_guard<std::mutex> lk(mutex_);
  last_compute_time_ = -1.0;
  for (auto & pc : plots_)
    for (auto & lc : pc.lines)
      lc.ring = RingBuffer(lc.ring.capacity);
}

void SimPlotter::compute(const mjModel *, mjData * d, int)
{
  if (!running_) return;
  if (++step_ % update_every_ != 0) return;

  // When MuJoCo is paused it keeps calling compute() via mj_forward() but
  // d->time does not advance.  Skip pushes to avoid flooding the ring buffers
  // with identical timestamps, which would wipe out all historical data.
  if (d->time == last_compute_time_) return;
  last_compute_time_ = d->time;

  // try_lock: never block physics; skip sample if render/service thread holds mutex
  std::unique_lock<std::mutex> lk(mutex_, std::try_to_lock);
  if (!lk.owns_lock()) return;

  for (auto & pc : plots_)
  {
    if (pc.paused) continue;
    for (auto & lc : pc.lines)
    {
      if (lc.paused || !lc.source.valid) continue;
      const double y_val = readSource(lc.source, d);
      const double x_val = (lc.is_scatter && lc.source_x.valid)
                             ? readSource(lc.source_x, d)
                             : d->time;
      lc.ring.push(x_val, y_val);
    }
  }
}

// ─── Render thread ────────────────────────────────────────────────────────────

void SimPlotter::startRenderThread()
{
  running_ = true;
  render_thread_ = std::thread(&SimPlotter::renderLoop, this);
}

void SimPlotter::stopThreads()
{
  stop_executor_ = true;
  running_       = false;

  if (render_thread_.joinable()) render_thread_.join();
  if (executor_) executor_->cancel();
  if (ros_thread_.joinable()) ros_thread_.join();
}

/** Snapshot of one line for rendering. */
struct LineSnap
{
  std::string         label;
  std::vector<double> times, values;  // X and Y; for line plots times=sim_time
  float               color[3];
  int                 yaxis;
  bool                is_scatter = false;
  std::string         x_label;   ///< X axis label for scatter (raw src_x string)
};
struct PlotSnap
{
  std::string            name, title;
  float                  x, y, w, h;
  bool                   auto_x, auto_y, auto_y2;
  double                 range_x[2], range_y[2], range_y2[2];
  bool                   has_y2;
  bool                   all_scatter = false; ///< true when every line is scatter
  std::vector<LineSnap>  lines;
};

// ─── Render backend selection ─────────────────────────────────────────────────

enum class RenderBackend { EGL_OPENGL, SOFTWARE };

namespace {

// Returns true if a shared library is available on this system.
// Uses RTLD_NOLOAD first (zero cost if already resident), then a file-system
// scan of standard library paths.  We deliberately NEVER load EGL/GL libraries
// here: their constructors run initialisation code (Mesa DRI2 auth, NVIDIA EGL
// device enumeration) that can leave global state that corrupts later context
// creation by SDL2.
bool libAvailable(const char * name)
{
  // Fast path: lib already loaded in this process
  void * h = dlopen(name, RTLD_LAZY | RTLD_LOCAL | RTLD_NOLOAD);
  if (h) { dlclose(h); return true; }

  // Slow path: look for the .so file on disk without loading it
  static const char * const prefixes[] = {
    "/usr/lib/x86_64-linux-gnu/",
    "/usr/lib/aarch64-linux-gnu/",
    "/usr/lib/arm-linux-gnueabihf/",
    "/usr/lib/",
    "/usr/local/lib/",
    nullptr
  };
  for (int i = 0; prefixes[i]; ++i) {
    std::string path = std::string(prefixes[i]) + name;
    if (access(path.c_str(), F_OK) == 0) return true;
  }
  return false;
}

struct SysCaps {
  int  sdl_major = 0, sdl_minor = 0, sdl_patch = 0;
  bool sdl_egl_hint = false;   // SDL_VIDEO_X11_FORCE_EGL (needs SDL >= 2.0.16)
  bool egl_present  = false;   // libEGL.so.1 found
  bool egl_nvidia   = false;   // libEGL_nvidia.so.0 (GLVND NVIDIA EGL)
  bool egl_mesa     = false;   // libEGL_mesa.so.0   (Mesa EGL / llvmpipe / iGPU)
  bool nvidia_driver = false;  // /proc/driver/nvidia/version readable
  bool on_x11       = false;
  bool on_wayland   = false;
};

SysCaps probeSysCaps()
{
  SysCaps c;

  SDL_version v;
  SDL_GetVersion(&v);
  c.sdl_major = v.major; c.sdl_minor = v.minor; c.sdl_patch = v.patch;
  // SDL_VIDEO_X11_FORCE_EGL was introduced in SDL 2.0.16 (2021-03-18)
  c.sdl_egl_hint = (v.major > 2 ||
                    (v.major == 2 && (v.minor > 0 ||
                                      (v.minor == 0 && v.patch >= 16))));

  c.egl_present  = libAvailable("libEGL.so.1");
  c.egl_nvidia   = libAvailable("libEGL_nvidia.so.0");
  c.egl_mesa     = libAvailable("libEGL_mesa.so.0");

  {
    FILE * f = fopen("/proc/driver/nvidia/version", "r");
    if (f) { c.nvidia_driver = true; fclose(f); }
  }

  c.on_x11     = (getenv("DISPLAY")          != nullptr);
  c.on_wayland = (getenv("WAYLAND_DISPLAY")  != nullptr);

  // ── Capability report ────────────────────────────────────────────────────
  fprintf(stderr,
    "[SimPlotter] ── Render capability probe ───────────────────────────────\n"
    "[SimPlotter]  SDL2 runtime:    %d.%d.%d  "
        "(EGL hint %s)\n"
    "[SimPlotter]  Display server:  %s%s%s\n"
    "[SimPlotter]  EGL libraries:   libEGL.so.1=%s  "
        "libEGL_nvidia=%s  libEGL_mesa=%s\n"
    "[SimPlotter]  NVIDIA driver:   %s\n",
    c.sdl_major, c.sdl_minor, c.sdl_patch,
    c.sdl_egl_hint ? "supported" : "NOT supported (needs >= 2.0.16)",
    c.on_x11 ? "X11 " : "", c.on_wayland ? "Wayland" : "",
    (!c.on_x11 && !c.on_wayland) ? "(none detected)" : "",
    c.egl_present  ? "yes" : "no",
    c.egl_nvidia   ? "yes" : "no",
    c.egl_mesa     ? "yes" : "no",
    c.nvidia_driver ? "yes" : "no/unknown");

  // ── Warnings for missing components ──────────────────────────────────────
  if (!c.egl_present)
    fprintf(stderr,
      "[SimPlotter]  WARN: libEGL.so.1 not found — GPU acceleration unavailable.\n"
      "[SimPlotter]        Install: sudo apt install libegl1\n");

  if (!c.sdl_egl_hint)
    fprintf(stderr,
      "[SimPlotter]  WARN: SDL2 %d.%d.%d < 2.0.16 — SDL_VIDEO_X11_FORCE_EGL\n"
      "[SimPlotter]        is not supported.  Update SDL2 for GPU acceleration.\n"
      "[SimPlotter]        Install: sudo apt install libsdl2-dev  (Ubuntu 22.04+)\n",
      c.sdl_major, c.sdl_minor, c.sdl_patch);

  if (c.nvidia_driver && !c.egl_nvidia)
    fprintf(stderr,
      "[SimPlotter]  WARN: NVIDIA driver detected but libEGL_nvidia.so.0 missing.\n"
      "[SimPlotter]        Install: sudo apt install libnvidia-egl-x11-1\n");

  if (!c.on_x11 && !c.on_wayland)
    fprintf(stderr,
      "[SimPlotter]  WARN: No display server detected (DISPLAY / WAYLAND_DISPLAY\n"
      "[SimPlotter]        unset).  Rendering will likely fail entirely.\n");

  fprintf(stderr,
    "[SimPlotter] ────────────────────────────────────────────────────────────\n");
  fflush(stderr);

  return c;
}

// Returns the best backend this system supports, logging the decision.
RenderBackend chooseBestBackend(const SysCaps & c)
{
  if (c.sdl_egl_hint && c.egl_present && (c.on_x11 || c.on_wayland)) {
    const char * via = c.egl_nvidia ? "NVIDIA EGL (GLVND)" :
                       c.egl_mesa   ? "Mesa EGL" : "EGL";
    fprintf(stderr, "[SimPlotter]  Backend: GPU/OpenGL via %s "
                    "(hardware-accelerated)\n", via);
    fflush(stderr);
    return RenderBackend::EGL_OPENGL;
  }

  fprintf(stderr,
    "[SimPlotter]  Backend: Software renderer (CPU / XShmPutImage).\n"
    "[SimPlotter]  Reason: %s%s%s\n",
    !c.sdl_egl_hint ? "SDL2 too old for EGL hint. " : "",
    !c.egl_present  ? "libEGL not found. "           : "",
    (!c.on_x11 && !c.on_wayland) ? "No display server. " : "");
  fflush(stderr);
  return RenderBackend::SOFTWARE;
}

}  // anonymous namespace

// ─── Render thread ────────────────────────────────────────────────────────────
// Runs in render_thread_.  Backend is chosen at runtime:
//   1st choice — SDL2 + EGL OpenGL 3.2 core (GPU, no GLX/X11 protocol conflict)
//   Fallback   — SDL2 software renderer (CPU, always works)

void SimPlotter::renderLoop()
{
  fprintf(stderr, "[SimPlotter DEBUG] renderLoop: thread started\n"); fflush(stderr);

  // ── Claim the render slot ─────────────────────────────────────────────────
  // Block until no other SimPlotter render thread is active, then take the slot.
  // This prevents concurrent ImGui/ImPlot context use that would crash on reload.
  RenderBackend backend;
  bool          use_nvidia_egl = false;   // captured for fallback cleanup
  {
    std::unique_lock<std::mutex> slot_lk(g_render_slot_mtx);
    g_render_slot_cv.wait(slot_lk, [] { return !g_render_slot_taken; });
    if (!running_) return;           // stopped while waiting — nothing to do
    g_render_slot_taken = true;

    // Probe system capabilities and choose backend (logged to stderr).
    const SysCaps caps = probeSysCaps();
    backend = chooseBestBackend(caps);

    // EGL attributes must be set before SDL_Init so the video subsystem
    // picks the right OpenGL platform layer.
    if (backend == RenderBackend::EGL_OPENGL) {
      setenv("SDL_VIDEO_X11_FORCE_EGL", "1", 1);

      // On NVIDIA/GLVND systems, Mesa EGL gets loaded alongside NVIDIA EGL.
      // Mesa's EGL constructor tries DRI2 auth which fails on NVIDIA hardware,
      // corrupting GLVND's vendor dispatch and causing eglMakeCurrent to fail.
      // Fix: pin GLVND to the NVIDIA EGL vendor JSON before SDL2 loads any EGL lib.
      if (caps.egl_nvidia) {
        use_nvidia_egl = true;
        setenv("__EGL_VENDOR_LIBRARY_FILENAMES",
               "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
               0 /* don't override if user already set it */);
      }

      SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
      SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
      SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
      SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
      SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);  // 2-D plots need no depth
    }

    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "1");
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: calling SDL_Init\n"); fflush(stderr);
    if (g_sdl_refcount == 0 && SDL_Init(SDL_INIT_VIDEO) != 0)
    {
      fprintf(stderr, "[SimPlotter] SDL_Init failed: %s\n", SDL_GetError());
      g_render_slot_taken = false;
      g_render_slot_cv.notify_all();
      return;
    }
    ++g_sdl_refcount;
  }
  fprintf(stderr, "[SimPlotter DEBUG] renderLoop: SDL_Init OK\n"); fflush(stderr);

  // ── SDL2 window + backend-specific context ────────────────────────────────
  auto release_slot = [&]() {
    std::lock_guard<std::mutex> slot_lk(g_render_slot_mtx);
    if (--g_sdl_refcount == 0) SDL_Quit();
    g_render_slot_taken = false;
    g_render_slot_cv.notify_all();
  };

  fprintf(stderr, "[SimPlotter DEBUG] renderLoop: calling SDL_CreateWindow\n"); fflush(stderr);
  const Uint32 win_flags_base = SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
  SDL_Window * window = SDL_CreateWindow(
      "SimPlotter",
      SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      win_w_, win_h_,
      win_flags_base | (backend == RenderBackend::EGL_OPENGL ? SDL_WINDOW_OPENGL : 0));
  if (!window)
  {
    fprintf(stderr, "[SimPlotter] SDL_CreateWindow failed: %s\n", SDL_GetError());
    release_slot();
    return;
  }
  fprintf(stderr, "[SimPlotter DEBUG] renderLoop: SDL_CreateWindow OK\n"); fflush(stderr);

  // Backend-specific handles (exactly one is non-null).
  SDL_GLContext  gl_ctx   = nullptr;
  SDL_Renderer * renderer = nullptr;

  if (backend == RenderBackend::EGL_OPENGL) {
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: creating EGL/OpenGL context\n"); fflush(stderr);
    gl_ctx = SDL_GL_CreateContext(window);
    if (!gl_ctx) {
      fprintf(stderr,
        "[SimPlotter] WARN: EGL context creation failed (%s).\n"
        "[SimPlotter]       Falling back to software renderer.\n"
        "[SimPlotter]       Tip: set __EGL_VENDOR_LIBRARY_FILENAMES to your vendor JSON\n"
        "[SimPlotter]       to force a specific EGL vendor if GLVND routing is wrong.\n",
        SDL_GetError());

      // SDL_GL_CreateContext leaves gl_config.driver_loaded = 1 even on failure.
      // SDL_GetWindowSurface() refuses to work when a GL driver is loaded, which
      // makes the software renderer's RenderPresent crash on the first frame.
      // Fix: unload the GL library (resets driver_loaded → 0), then destroy the
      // EGL-tainted window, and create a fresh non-OpenGL window for software rendering.
      SDL_GL_UnloadLibrary();
      SDL_DestroyWindow(window);
      window = nullptr;

      unsetenv("SDL_VIDEO_X11_FORCE_EGL");
      if (use_nvidia_egl) unsetenv("__EGL_VENDOR_LIBRARY_FILENAMES");
      SDL_GL_ResetAttributes();
      backend = RenderBackend::SOFTWARE;

      // Fresh window with no OpenGL flag — software renderer works on this cleanly.
      window = SDL_CreateWindow(
          "SimPlotter",
          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
          win_w_, win_h_,
          win_flags_base);
      if (!window) {
        fprintf(stderr, "[SimPlotter] SDL_CreateWindow (software fallback) failed: %s\n",
                SDL_GetError());
        release_slot();
        return;
      }
      fprintf(stderr, "[SimPlotter DEBUG] renderLoop: software fallback window created OK\n");
      fflush(stderr);
    } else {
      SDL_GL_MakeCurrent(window, gl_ctx);
      SDL_GL_SetSwapInterval(0);  // uncapped — our sleep controls fps
    }
  }

  if (backend == RenderBackend::SOFTWARE) {
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: calling SDL_CreateRenderer (SOFTWARE)\n"); fflush(stderr);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer) {
      fprintf(stderr, "[SimPlotter] SDL_CreateRenderer failed: %s\n", SDL_GetError());
      SDL_DestroyWindow(window);
      release_slot();
      return;
    }
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: SDL_CreateRenderer OK\n"); fflush(stderr);
  }

  // ── ImGui + ImPlot init ───────────────────────────────────────────────────
  IMGUI_CHECKVERSION();
  ImGuiContext * imgui_ctx  = ImGui::CreateContext();
  ImPlotContext* implot_ctx = ImPlot::CreateContext();
  ImGui::SetCurrentContext(imgui_ctx);
  ImPlot::SetCurrentContext(implot_ctx);

  ImGuiIO & io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  ImPlot::StyleColorsDark();
  if (backend == RenderBackend::EGL_OPENGL) {
    ImGui_ImplSDL2_InitForOpenGL(window, gl_ctx);
    ImGui_ImplOpenGL3_Init("#version 150");
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: ImGui/OpenGL3 init OK — GPU path\n");
  } else {
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);
    fprintf(stderr, "[SimPlotter DEBUG] renderLoop: ImGui/SDLRenderer2 init OK — software path\n");
  }
  fprintf(stderr, "[SimPlotter DEBUG] renderLoop: ImGui init complete — starting render loop\n"); fflush(stderr);

  // ── Render loop ───────────────────────────────────────────────────────────
  std::vector<PlotSnap> snaps;

  struct CursorLock { bool locked = false; double t = 0.0; };
  std::map<std::string, CursorLock> cursor_locks;

  // Per-plot UI state: needs_fit triggers a one-shot SetNextAxesToFit();
  // paused freezes the display snapshot while physics keeps running.
  struct PlotUiState { bool needs_fit = true; bool paused = false; };
  std::map<std::string, PlotUiState> plot_ui;

  // Drag-skip: on the software path, XPutImage during a window drag competes
  // with GLFW's glXSwapBuffers on the same X server and stalls physics.
  // On the EGL path this is not a concern (eglSwapBuffers uses DRI3/Present,
  // a separate X11 extension channel), but we skip redundant frames anyway.
  bool     window_moving   = false;
  auto     last_move_time  = std::chrono::steady_clock::now();

  while (running_)
  {
    // Each frame must restore this thread's ImGui/ImPlot context — another
    // SimPlotter instance running concurrently may have changed the globals.
    ImGui::SetCurrentContext(imgui_ctx);
    ImPlot::SetCurrentContext(implot_ctx);

    // ── SDL2 event processing ─────────────────────────────────────────────
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) running_ = false;
      if (event.type == SDL_WINDOWEVENT)
      {
        if (event.window.event == SDL_WINDOWEVENT_CLOSE) running_ = false;
        if (event.window.event == SDL_WINDOWEVENT_MOVED)
        {
          window_moving  = true;
          last_move_time = std::chrono::steady_clock::now();
        }
      }
    }
    if (!running_) break;

    // ── Skip rendering while dragging — do not send XPutImage to X server ──
    if (window_moving)
    {
      using ms = std::chrono::milliseconds;
      const auto idle = std::chrono::duration_cast<ms>(
          std::chrono::steady_clock::now() - last_move_time);
      if (idle.count() < 200)
      {
        std::this_thread::sleep_for(ms(16));
        continue;   // no X11 calls this frame
      }
      window_moving = false;  // settled — resume rendering
    }

    // ── Build snapshot from ring buffers (mutex try_lock — never stall physics) ──
    {
      std::unique_lock<std::mutex> lk(mutex_, std::try_to_lock);
      if (lk.owns_lock())
      {
        // Resize to match current plots count; if size changed rebuild all.
        const bool size_changed = (snaps.size() != plots_.size());
        if (size_changed) snaps.resize(plots_.size());

        for (size_t pi = 0; pi < plots_.size(); ++pi)
        {
          const auto & pc = plots_[pi];
          // Skip rebuilding paused plots — they keep their previous snapshot.
          if (!size_changed && plot_ui[pc.name].paused) continue;

          PlotSnap ps;
          ps.name   = pc.name;   ps.title  = pc.title;
          ps.x      = pc.init_x; ps.y      = pc.init_y;
          ps.w      = pc.init_w; ps.h      = pc.init_h;
          ps.auto_x = pc.auto_range_x;
          ps.auto_y = pc.auto_range_y;
          ps.auto_y2 = pc.auto_range_y2;
          std::memcpy(ps.range_x,  pc.range_x,  sizeof(ps.range_x));
          std::memcpy(ps.range_y,  pc.range_y,  sizeof(ps.range_y));
          std::memcpy(ps.range_y2, pc.range_y2, sizeof(ps.range_y2));
          ps.has_y2 = pc.has_y2;
          bool all_scat = !pc.lines.empty();
          for (const auto & lc : pc.lines)
          {
            LineSnap ls;
            ls.label      = lc.label;
            ls.yaxis      = lc.yaxis;
            ls.color[0]   = lc.color[0];
            ls.color[1]   = lc.color[1];
            ls.color[2]   = lc.color[2];
            ls.is_scatter = lc.is_scatter;
            ls.x_label    = lc.is_scatter ? lc.source_x.raw : "";
            lc.ring.copyOrdered(ls.times, ls.values);
            ps.lines.push_back(std::move(ls));
            if (!lc.is_scatter) all_scat = false;
          }
          ps.all_scatter = all_scat;
          snaps[pi] = std::move(ps);
        }
      } // lock released here — physics thread can run freely
    } // try_lock scope

    // ── ImGui frame ───────────────────────────────────────────────────────
    if (backend == RenderBackend::EGL_OPENGL) ImGui_ImplOpenGL3_NewFrame();
    else                                      ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    int cur_win_w, cur_win_h;
    SDL_GetWindowSize(window, &cur_win_w, &cur_win_h);
    const float fw = static_cast<float>(cur_win_w);
    const float fh = static_cast<float>(cur_win_h);

    ImGui::SetNextWindowPos (ImVec2(0, 0),   ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(fw, fh), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,   ImVec2(4, 4));
    ImGui::Begin("##host",
                 nullptr,
                 ImGuiWindowFlags_NoTitleBar    | ImGuiWindowFlags_NoResize   |
                 ImGuiWindowFlags_NoMove        | ImGuiWindowFlags_NoScrollbar |
                 ImGuiWindowFlags_NoSavedSettings);

    const int   n_plots = static_cast<int>(snaps.size());
    const int   cols    = (n_plots <= 3) ? n_plots
                          : static_cast<int>(std::ceil(std::sqrt(static_cast<float>(n_plots))));
    const int   rows    = (n_plots + cols - 1) / cols;
    const float cell_w  = (fw - 4.f * (cols + 1)) / static_cast<float>(cols);
    const float cell_h  = (fh - 4.f * (rows + 1) - ImGui::GetFrameHeight())
                          / static_cast<float>(rows);

    for (int i = 0; i < n_plots; ++i)
    {
      const auto & ps  = snaps[i];
      const int    col = i % cols;
      const int    row = i / cols;

      if (col > 0) ImGui::SameLine();

      ImGui::BeginGroup();
      PlotUiState & ui = plot_ui[ps.name];

      // Control buttons + title on one line
      ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(3, 1));
      if (ImGui::SmallButton(ui.paused ? "Run " : "Pause"))
        ui.paused = !ui.paused;
      ImGui::SameLine(0, 4);
      if (ImGui::SmallButton("Fit"))
        ui.needs_fit = true;
      ImGui::PopStyleVar();
      ImGui::SameLine(0, 8);
      ImGui::TextDisabled("%s", ps.title.c_str());

      ImVec2 plot_size(cell_w, cell_h - ImGui::GetTextLineHeightWithSpacing());
      (void)row;

      // Queue a one-shot axis fit (cleared after use so user zoom is preserved).
      if (ui.needs_fit) { ImPlot::SetNextAxesToFit(); ui.needs_fit = false; }
      if (ImPlot::BeginPlot(("##plot_" + ps.name).c_str(), plot_size,
                             ImPlotFlags_Crosshairs))
      {
        // Y-axis title: only show label when exactly one line is on that axis.
        // With multiple lines the concatenated string is too long when rotated 90°;
        // the legend already shows each line's name and color.
        auto buildAxisLabel = [&](int axis_id) -> std::string
        {
          std::string lbl;
          int count = 0;
          for (const auto & ls : ps.lines)
          {
            if (ls.yaxis != axis_id) continue;
            ++count;
            lbl = ls.label.size() > 20 ? ls.label.substr(0, 17) + "..." : ls.label;
          }
          return (count == 1) ? lbl : "";
        };
        const std::string y1_lbl = buildAxisLabel(1);
        const std::string y2_lbl = buildAxisLabel(2);

        // ── Axis setup ───────────────────────────────────────────────────
        // For scatter plots derive the X axis label from the first scatter line's src_x
        std::string x_axis_label = "Time (s)";
        if (ps.all_scatter)
        {
          for (const auto & ls : ps.lines)
            if (ls.is_scatter && !ls.x_label.empty()) { x_axis_label = ls.x_label; break; }
        }

        // Axis setup: explicit limits use ImGuiCond_Once so the user can zoom
        // freely after the initial view.  AutoFit is NOT used — instead a
        // one-shot SetNextAxesToFit() is queued via needs_fit (above) so zoom
        // is never clobbered every frame.
        ImPlot::SetupAxis(ImAxis_X1, x_axis_label.c_str());
        if (!ps.auto_x)
          ImPlot::SetupAxisLimits(ImAxis_X1, ps.range_x[0], ps.range_x[1], ImGuiCond_Once);
        ImPlot::SetupAxisFormat(ImAxis_X1, "%.2f");  // fixed width: no jumping

        ImPlot::SetupAxis(ImAxis_Y1, y1_lbl.c_str());
        if (!ps.auto_y)
          ImPlot::SetupAxisLimits(ImAxis_Y1, ps.range_y[0], ps.range_y[1], ImGuiCond_Once);
        ImPlot::SetupAxisFormat(ImAxis_Y1, "%.3f");  // fixed width

        if (ps.has_y2)
        {
          ImPlot::SetupAxis(ImAxis_Y2, y2_lbl.c_str(), ImPlotAxisFlags_AuxDefault);
          if (!ps.auto_y2)
            ImPlot::SetupAxisLimits(ImAxis_Y2, ps.range_y2[0], ps.range_y2[1], ImGuiCond_Once);
          ImPlot::SetupAxisFormat(ImAxis_Y2, "%.3f");  // fixed width
        }

        // ── Plot lines ───────────────────────────────────────────────────
        for (const auto & ls : ps.lines)
        {
          if (ls.times.empty()) continue;
          ImPlot::SetAxes(ImAxis_X1, ls.yaxis == 2 ? ImAxis_Y2 : ImAxis_Y1);
          ImPlot::SetNextLineStyle(ImVec4(ls.color[0], ls.color[1], ls.color[2], 1.0f));
          if (ls.is_scatter)
            ImPlot::PlotScatter(ls.label.c_str(),
                                ls.times.data(), ls.values.data(),
                                static_cast<int>(ls.times.size()));
          else
            ImPlot::PlotLine(ls.label.c_str(),
                             ls.times.data(), ls.values.data(),
                             static_cast<int>(ls.times.size()));
        }

        // ── Cursor: hover + click-to-lock (line plots only) ──────────────
        CursorLock & cl = cursor_locks[ps.name];
        if (ps.all_scatter)
        {
          // For scatter plots, rely on ImPlot's built-in crosshair (ImPlotFlags_Crosshairs)
          ImPlot::EndPlot();
          ImGui::EndGroup();
          if (col == cols - 1 && row < rows - 1) ImGui::Spacing();
          continue;
        }

        if (ImPlot::IsPlotHovered() &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
          if (cl.locked)
            cl.locked = false;                            // any click unlocks
          else
          { cl.locked = true; cl.t = ImPlot::GetPlotMousePos(ImAxis_X1).x; }
        }

        const bool   show_cursor = cl.locked || ImPlot::IsPlotHovered();
        const double cursor_t    = cl.locked
                                     ? cl.t
                                     : (ImPlot::IsPlotHovered()
                                          ? ImPlot::GetPlotMousePos(ImAxis_X1).x
                                          : 0.0);

        if (show_cursor)
        {
          auto interpAt = [](const std::vector<double> & ts,
                             const std::vector<double> & vs,
                             double t) -> double
          {
            if (ts.empty()) return 0.0;
            if (t <= ts.front()) return vs.front();
            if (t >= ts.back())  return vs.back();
            auto it = std::lower_bound(ts.begin(), ts.end(), t);
            const size_t hi = static_cast<size_t>(it - ts.begin());
            const size_t lo = hi - 1;
            const double alpha = (ts[hi] > ts[lo])
                                   ? (t - ts[lo]) / (ts[hi] - ts[lo])
                                   : 0.0;
            return vs[lo] + alpha * (vs[hi] - vs[lo]);
          };

          ImDrawList * dl = ImPlot::GetPlotDrawList();

          auto drawDashed = [&](ImVec2 p1, ImVec2 p2, ImU32 col,
                                float dash = 5.f, float gap = 3.f)
          {
            const float dx = p2.x - p1.x, dy = p2.y - p1.y;
            const float len = std::sqrt(dx * dx + dy * dy);
            if (len < 0.5f) return;
            const float nx = dx / len, ny = dy / len;
            for (float t = 0.f; t < len; t += dash + gap)
            {
              const float t2 = std::min(t + dash, len);
              dl->AddLine(ImVec2(p1.x + nx * t,  p1.y + ny * t),
                          ImVec2(p1.x + nx * t2, p1.y + ny * t2),
                          col, 1.2f);
            }
          };

          const ImVec2 plot_pos  = ImPlot::GetPlotPos();
          const ImVec2 plot_size = ImPlot::GetPlotSize();
          const float  x_axis_y  = plot_pos.y + plot_size.y;
          const float  y1_axis_x = plot_pos.x;
          const float  y2_axis_x = plot_pos.x + plot_size.x;

          if (cl.locked)
          {
            const ImVec2 top = ImPlot::PlotToPixels(cursor_t,
                ImPlot::GetPlotLimits(ImAxis_X1, ImAxis_Y1).Y.Max, ImAxis_X1, ImAxis_Y1);
            const ImVec2 bot = ImVec2(top.x, x_axis_y);
            dl->AddLine(top, bot, IM_COL32(255, 255, 100, 200), 1.5f);
          }

          ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
          ImPlot::TagX(cursor_t,
                       cl.locked ? ImVec4(1.f, 1.f, 0.4f, 1.f)
                                 : ImVec4(0.95f, 0.95f, 0.95f, 1.f),
                       "%.3f", cursor_t);

          for (const auto & ls : ps.lines)
          {
            if (ls.times.empty()) continue;

            const ImAxis yax   = (ls.yaxis == 2) ? ImAxis_Y2 : ImAxis_Y1;
            const float  yax_x = (ls.yaxis == 2) ? y2_axis_x : y1_axis_x;
            const double v     = interpAt(ls.times, ls.values, cursor_t);
            const ImVec4 col4  (ls.color[0], ls.color[1], ls.color[2], 1.f);
            const ImU32  col32 = ImGui::ColorConvertFloat4ToU32(col4);

            const ImVec2 pt = ImPlot::PlotToPixels(cursor_t, v, ImAxis_X1, yax);

            drawDashed(pt, ImVec2(yax_x, pt.y), col32);
            drawDashed(pt, ImVec2(pt.x, x_axis_y), col32);

            dl->AddCircleFilled(pt, 6.f, col32);
            dl->AddCircle      (pt, 6.f, IM_COL32(255, 255, 255, 220), 0, 1.5f);
            dl->AddCircleFilled(pt, 2.5f, IM_COL32(255, 255, 255, 255));

            ImPlot::SetAxes(ImAxis_X1, yax);
            ImPlot::TagY(v, col4, "%.3f", v);
          }

          if (!cl.locked && ImPlot::IsPlotHovered())
          {
            ImGui::BeginTooltip();
            ImGui::TextDisabled("t = %.5f s  [click to lock]", cursor_t);
            ImGui::Separator();
            for (const auto & ls : ps.lines)
            {
              if (ls.times.empty()) continue;
              const double v = interpAt(ls.times, ls.values, cursor_t);
              ImGui::TextColored(
                  ImVec4(ls.color[0], ls.color[1], ls.color[2], 1.f),
                  "%-28s  %+.6g", ls.label.c_str(), v);
            }
            ImGui::EndTooltip();
          }
        }

        ImPlot::EndPlot();
      }
      ImGui::EndGroup();

      if (col == cols - 1 && row < rows - 1)
        ImGui::Spacing();
    }

    ImGui::End();
    ImGui::PopStyleVar(2);

    // ── Present ───────────────────────────────────────────────────────────
    ImGui::Render();
    if (backend == RenderBackend::EGL_OPENGL) {
      int vp_w, vp_h;
      SDL_GL_GetDrawableSize(window, &vp_w, &vp_h);
      glViewport(0, 0, vp_w, vp_h);
      glClearColor(0.117f, 0.117f, 0.117f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window);
    } else {
      SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
      SDL_RenderClear(renderer);
      ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
      SDL_RenderPresent(renderer);
    }

    // ~60 fps cap
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  // ── Cleanup + release slot ────────────────────────────────────────────────
  ImGui::SetCurrentContext(imgui_ctx);
  ImPlot::SetCurrentContext(implot_ctx);
  if (backend == RenderBackend::EGL_OPENGL) {
    ImGui_ImplOpenGL3_Shutdown();
    SDL_GL_DeleteContext(gl_ctx);
  } else {
    ImGui_ImplSDLRenderer2_Shutdown();
    SDL_DestroyRenderer(renderer);
  }
  ImGui_ImplSDL2_Shutdown();
  ImPlot::DestroyContext(implot_ctx);
  ImGui::DestroyContext(imgui_ctx);
  SDL_DestroyWindow(window);
  release_slot();
}


// ─── ROS 2 service thread ─────────────────────────────────────────────────────

void SimPlotter::startRosThread(const std::string & node_name)
{
  if (!rclcpp::ok())
  {
    int    argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  node_ = rclcpp::Node::make_shared("sim_plotter_" + node_name, opts);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  cmd_srv_ = node_->create_service<PlotCmd>(
      "~/" + node_name + "/plot_command",
      [this](PlotCmdReq req, PlotCmdRes res) { handlePlotCommand(req, res); });

  ros_thread_ = std::thread([this] {
    while (!stop_executor_ && rclcpp::ok())
      executor_->spin_once(std::chrono::milliseconds(100));
  });
}

// ─── PlotCommand service handler ─────────────────────────────────────────────

void SimPlotter::handlePlotCommand(PlotCmdReq req, PlotCmdRes res)
{
  const auto & op   = req->operation;
  const auto & plot = req->plot;
  const auto & line = req->line;
  const auto   kv   = parseArgs(req->args);

  std::lock_guard<std::mutex> lk(mutex_);

  auto findPlot = [&](const std::string & name) -> PlotConfig * {
    for (auto & pc : plots_)
      if (pc.name == name) return &pc;
    return nullptr;
  };

  // ── list ─────────────────────────────────────────────────────────────────
  if (op == "list")
  {
    std::ostringstream j;
    j << "[";
    for (size_t i = 0; i < plots_.size(); ++i)
    {
      const auto & pc = plots_[i];
      j << "{\"name\":\"" << pc.name << "\",\"title\":\"" << pc.title
        << "\",\"lines\":[";
      for (size_t k = 0; k < pc.lines.size(); ++k)
        j << "\"" << pc.lines[k].label << "\""
          << (k + 1 < pc.lines.size() ? "," : "");
      j << "]}" << (i + 1 < plots_.size() ? "," : "");
    }
    j << "]";
    res->success = true;
    res->message = j.str();
    return;
  }

  // ── add_plot ─────────────────────────────────────────────────────────────
  if (op == "add_plot")
  {
    if (plot.empty()) { res->message = "plot name required"; return; }
    if (findPlot(plot)) { res->message = "plot '" + plot + "' already exists"; return; }
    PlotConfig pc;
    pc.name  = plot;
    pc.title = kv.count("title") ? kv.at("title") : plot;
    applyPlotFromArgs(pc, kv);
    plots_.push_back(std::move(pc));
    res->success = true;
    res->message = "plot '" + plot + "' added";
    return;
  }

  // ── del_plot ─────────────────────────────────────────────────────────────
  if (op == "del_plot")
  {
    auto it = std::remove_if(plots_.begin(), plots_.end(),
                             [&](const PlotConfig & pc) { return pc.name == plot; });
    if (it == plots_.end()) { res->message = "plot '" + plot + "' not found"; return; }
    plots_.erase(it, plots_.end());
    res->success = true;
    res->message = "plot '" + plot + "' removed";
    return;
  }

  // ── mod_plot ─────────────────────────────────────────────────────────────
  if (op == "mod_plot")
  {
    auto * pc = findPlot(plot);
    if (!pc) { res->message = "plot '" + plot + "' not found"; return; }
    applyPlotFromArgs(*pc, kv);
    res->success = true;
    res->message = "plot '" + plot + "' updated";
    return;
  }

  // ── add_line ─────────────────────────────────────────────────────────────
  if (op == "add_line")
  {
    auto * pc = findPlot(plot);
    if (!pc) { res->message = "plot '" + plot + "' not found"; return; }
    if (line.empty()) { res->message = "line label required"; return; }

    LineConfig lc;
    lc.label = line;
    lc.ring  = RingBuffer(500);
    applyLineFromArgs(lc, kv);
    if (lc.source.type == DataSourceType::Unknown)
      { res->message = "invalid or missing src"; return; }

    // Auto-color if not specified
    if (!kv.count("color"))
      autoColor(static_cast<int>(pc->lines.size()), lc.color);

    if (lc.yaxis == 2) pc->has_y2 = true;
    pc->lines.push_back(std::move(lc));
    res->success = true;
    res->message = "line '" + line + "' added to plot '" + plot + "'";
    return;
  }

  // ── del_line ─────────────────────────────────────────────────────────────
  if (op == "del_line")
  {
    auto * pc = findPlot(plot);
    if (!pc) { res->message = "plot '" + plot + "' not found"; return; }
    auto it = std::remove_if(pc->lines.begin(), pc->lines.end(),
                             [&](const LineConfig & lc) { return lc.label == line; });
    if (it == pc->lines.end()) { res->message = "line '" + line + "' not found"; return; }
    pc->lines.erase(it, pc->lines.end());
    res->success = true;
    res->message = "line '" + line + "' removed";
    return;
  }

  // ── mod_line ─────────────────────────────────────────────────────────────
  if (op == "mod_line")
  {
    auto * pc = findPlot(plot);
    if (!pc) { res->message = "plot '" + plot + "' not found"; return; }
    for (auto & lc : pc->lines)
    {
      if (lc.label == line)
      {
        applyLineFromArgs(lc, kv);
        if (lc.yaxis == 2) pc->has_y2 = true;
        res->success = true;
        res->message = "line '" + line + "' updated";
        return;
      }
    }
    res->message = "line '" + line + "' not found";
    return;
  }

  // ── clear ─────────────────────────────────────────────────────────────────
  if (op == "clear")
  {
    int count = 0;
    for (auto & pc : plots_)
    {
      if (!plot.empty() && pc.name != plot) continue;
      for (auto & lc : pc.lines)
        lc.ring = RingBuffer(lc.ring.capacity);
      ++count;
    }
    res->success = true;
    res->message = std::to_string(count) + " plot(s) cleared";
    return;
  }

  // ── pause / resume ────────────────────────────────────────────────────────
  if (op == "pause" || op == "resume")
  {
    const bool do_pause = (op == "pause");
    for (auto & pc : plots_)
    {
      if (!plot.empty() && pc.name != plot) continue;
      if (line.empty()) { pc.paused = do_pause; }
      else
      {
        for (auto & lc : pc.lines)
          if (lc.label == line) lc.paused = do_pause;
      }
    }
    res->success = true;
    res->message = op + " applied";
    return;
  }

  // ── export ────────────────────────────────────────────────────────────────
  if (op == "export")
  {
    auto * pc = findPlot(plot);
    if (!pc) { res->message = "plot '" + plot + "' not found"; return; }

    std::string path = kv.count("path") ? kv.at("path") : ("/tmp/simplotter_" + plot + ".csv");
    std::ofstream f(path);
    if (!f) { res->message = "cannot open '" + path + "'"; return; }

    // Header
    f << "time";
    for (const auto & lc : pc->lines) f << "," << lc.label;
    f << "\n";

    // Data: align on time axis (use first line's times as reference)
    if (!pc->lines.empty())
    {
      std::vector<std::vector<double>> all_t(pc->lines.size()), all_v(pc->lines.size());
      for (size_t i = 0; i < pc->lines.size(); ++i)
        pc->lines[i].ring.copyOrdered(all_t[i], all_v[i]);

      const size_t rows = all_t[0].size();
      for (size_t r = 0; r < rows; ++r)
      {
        f << std::setprecision(6) << all_t[0][r];
        for (size_t c = 0; c < pc->lines.size(); ++c)
        {
          f << ",";
          if (r < all_v[c].size()) f << all_v[c][r];
        }
        f << "\n";
      }
    }
    res->success = true;
    res->message = "exported to " + path;
    return;
  }

  res->message = "unknown operation '" + op + "'";
}

}  // namespace MujocoRosUtils
