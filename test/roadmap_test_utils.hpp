#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>

namespace mujoco_ros_utils::test
{

inline std::filesystem::path xml_directory()
{
  if (const char *value = std::getenv("MUJOCO_ROS_UTILS_TEST_XML_DIR"))
  {
    return value;
  }
#ifdef MUJOCO_ROS_UTILS_TEST_XML_DIR
  return MUJOCO_ROS_UTILS_TEST_XML_DIR;
#else
  return std::filesystem::path(__FILE__).parent_path().parent_path() / "xml";
#endif
}

inline std::string read_file(const std::filesystem::path &path)
{
  std::ifstream stream(path);
  if (!stream)
  {
    throw std::runtime_error("Unable to read fixture: " + path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

inline hardware_interface::HardwareInfo hardware_info(
  const std::string &fixture, const std::string &name = {})
{
  const auto infos = hardware_interface::parse_control_resources_from_urdf(
    read_file(xml_directory() / fixture));
  if (name.empty())
  {
    if (infos.size() != 1)
    {
      throw std::runtime_error("Expected exactly one ros2_control component");
    }
    return infos.front();
  }
  for (const auto &info : infos)
  {
    if (info.name == name)
    {
      return info;
    }
  }
  throw std::runtime_error("ros2_control component not found: " + name);
}

class MujocoModel
{
public:
  explicit MujocoModel(const std::string &fixture)
  {
    char error[1024] = {};
    model_ = mj_loadXML((xml_directory() / fixture).c_str(), nullptr, error, sizeof(error));
    if (!model_)
    {
      throw std::runtime_error("MuJoCo failed to load fixture: " + std::string(error));
    }
    data_ = mj_makeData(model_);
    if (!data_)
    {
      mj_deleteModel(model_);
      model_ = nullptr;
      throw std::runtime_error("MuJoCo failed to allocate mjData");
    }
  }

  ~MujocoModel()
  {
    mj_deleteData(data_);
    mj_deleteModel(model_);
  }

  MujocoModel(const MujocoModel &) = delete;
  MujocoModel &operator=(const MujocoModel &) = delete;

  mjModel *model() const { return model_; }
  mjData *data() const { return data_; }

private:
  mjModel *model_{nullptr};
  mjData *data_{nullptr};
};

}  // namespace mujoco_ros_utils::test
