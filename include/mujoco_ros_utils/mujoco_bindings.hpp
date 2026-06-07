#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace MujocoRosUtils
{

class MujocoSensorBinding
{
public:
  MujocoSensorBinding() = default;

  MujocoSensorBinding(const mjModel * model, const std::string & name)
  {
    if(!model)
    {
      throw std::invalid_argument("Cannot bind MuJoCo sensor '" + name + "': model is null");
    }
    const int id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
    if(id < 0)
    {
      throw std::invalid_argument("MuJoCo sensor '" + name + "' does not exist");
    }
    bind(model, id);
  }

  MujocoSensorBinding(const mjModel * model, int id)
  {
    bind(model, id);
  }

  int id() const noexcept { return id_; }
  int address() const noexcept { return address_; }
  int dimension() const noexcept { return dimension_; }
  int type() const noexcept { return type_; }
  const std::string & name() const noexcept { return name_; }

  double read(const mjData * data, std::size_t index = 0) const
  {
    validate_data(data);
    if(index >= static_cast<std::size_t>(dimension_))
    {
      throw std::out_of_range(
          "Element " + std::to_string(index) + " is outside MuJoCo sensor '" + name_
          + "' dimension " + std::to_string(dimension_));
    }
    return static_cast<double>(data->sensordata[address_ + static_cast<int>(index)]);
  }

  void read(const mjData * data, double * output, std::size_t count) const
  {
    validate_data(data);
    if(!output)
    {
      throw std::invalid_argument("Output buffer for MuJoCo sensor '" + name_ + "' is null");
    }
    if(count > static_cast<std::size_t>(dimension_))
    {
      throw std::out_of_range(
          "Requested " + std::to_string(count) + " values from MuJoCo sensor '" + name_
          + "' with dimension " + std::to_string(dimension_));
    }
    for(std::size_t i = 0; i < count; ++i)
    {
      output[i] = static_cast<double>(data->sensordata[address_ + static_cast<int>(i)]);
    }
  }

private:
  void bind(const mjModel * model, int id)
  {
    if(!model)
    {
      throw std::invalid_argument("Cannot bind MuJoCo sensor: model is null");
    }
    if(id < 0 || id >= model->nsensor)
    {
      throw std::out_of_range("Invalid MuJoCo sensor ID " + std::to_string(id));
    }
    const char * sensor_name = mj_id2name(model, mjOBJ_SENSOR, id);
    if(!sensor_name || sensor_name[0] == '\0')
    {
      throw std::invalid_argument("MuJoCo sensor ID " + std::to_string(id) + " is unnamed");
    }
    model_ = model;
    id_ = id;
    address_ = model->sensor_adr[id];
    dimension_ = model->sensor_dim[id];
    type_ = model->sensor_type[id];
    name_ = sensor_name;
    if(address_ < 0 || dimension_ <= 0)
    {
      throw std::invalid_argument("MuJoCo sensor '" + name_ + "' has invalid storage metadata");
    }
  }

  void validate_data(const mjData * data) const
  {
    if(!model_ || id_ < 0)
    {
      throw std::logic_error("MuJoCo sensor binding is not initialized");
    }
    if(!data)
    {
      throw std::invalid_argument("Cannot read MuJoCo sensor '" + name_ + "': data is null");
    }
  }

  const mjModel * model_ = nullptr;
  int id_ = -1;
  int address_ = -1;
  int dimension_ = 0;
  int type_ = -1;
  std::string name_;
};

class MujocoActuatorBinding
{
public:
  MujocoActuatorBinding() = default;

  MujocoActuatorBinding(const mjModel * model, const std::string & name)
  {
    if(!model)
    {
      throw std::invalid_argument("Cannot bind MuJoCo actuator '" + name + "': model is null");
    }
    const int id = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    if(id < 0)
    {
      throw std::invalid_argument("MuJoCo actuator '" + name + "' does not exist");
    }
    bind(model, id, true);
  }

  MujocoActuatorBinding(const mjModel * model, int id)
  {
    bind(model, id, false);
  }

  int id() const noexcept { return id_; }
  const std::string & name() const noexcept { return name_; }
  bool control_limited() const noexcept { return control_limited_; }
  double minimum() const noexcept { return minimum_; }
  double maximum() const noexcept { return maximum_; }

  double read_control(const mjData * data) const
  {
    validate_data(data);
    return static_cast<double>(data->ctrl[id_]);
  }

  double read_force(const mjData * data) const
  {
    validate_data(data);
    return static_cast<double>(data->actuator_force[id_]);
  }

  double clamp(double value, double minimum = std::numeric_limits<double>::lowest(),
               double maximum = std::numeric_limits<double>::max()) const
  {
    if(!std::isfinite(value))
    {
      throw std::invalid_argument("Command for MuJoCo actuator '" + name_ + "' is not finite");
    }
    if(minimum > maximum)
    {
      throw std::invalid_argument("Invalid command range for MuJoCo actuator '" + name_ + "'");
    }
    if(control_limited_)
    {
      minimum = std::max(minimum, minimum_);
      maximum = std::min(maximum, maximum_);
    }
    if(minimum > maximum)
    {
      throw std::invalid_argument(
          "Command range does not overlap ctrlrange for MuJoCo actuator '" + name_ + "'");
    }
    return std::clamp(value, minimum, maximum);
  }

  void write(mjData * data, double value,
             double minimum = std::numeric_limits<double>::lowest(),
             double maximum = std::numeric_limits<double>::max()) const
  {
    validate_data(data);
    data->ctrl[id_] = static_cast<mjtNum>(clamp(value, minimum, maximum));
  }

private:
  void bind(const mjModel * model, int id, bool require_name)
  {
    if(!model)
    {
      throw std::invalid_argument("Cannot bind MuJoCo actuator: model is null");
    }
    if(id < 0 || id >= model->nu)
    {
      throw std::out_of_range("Invalid MuJoCo actuator ID " + std::to_string(id));
    }
    const char * actuator_name = mj_id2name(model, mjOBJ_ACTUATOR, id);
    if(require_name && (!actuator_name || actuator_name[0] == '\0'))
    {
      throw std::invalid_argument("MuJoCo actuator ID " + std::to_string(id) + " is unnamed");
    }
    model_ = model;
    id_ = id;
    name_ = actuator_name && actuator_name[0] != '\0'
              ? actuator_name
              : "unnamed_actuator_" + std::to_string(id);
    control_limited_ = model->actuator_ctrllimited[id] != 0;
    if(control_limited_)
    {
      minimum_ = static_cast<double>(model->actuator_ctrlrange[2 * id]);
      maximum_ = static_cast<double>(model->actuator_ctrlrange[2 * id + 1]);
      if(minimum_ > maximum_)
      {
        throw std::invalid_argument("MuJoCo actuator '" + name_ + "' has an invalid ctrlrange");
      }
    }
  }

  void validate_data(const mjData * data) const
  {
    if(!model_ || id_ < 0)
    {
      throw std::logic_error("MuJoCo actuator binding is not initialized");
    }
    if(!data)
    {
      throw std::invalid_argument("Cannot access MuJoCo actuator '" + name_ + "': data is null");
    }
  }

  const mjModel * model_ = nullptr;
  int id_ = -1;
  std::string name_;
  bool control_limited_ = false;
  double minimum_ = std::numeric_limits<double>::lowest();
  double maximum_ = std::numeric_limits<double>::max();
};

} // namespace MujocoRosUtils
