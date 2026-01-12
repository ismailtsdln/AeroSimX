/**
 * @file python_bindings.cpp
 * @brief Python bindings for AeroSimX core using pybind11
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "aerosimx/aerosimx.hpp"

namespace py = pybind11;
using namespace aerosimx;

PYBIND11_MODULE(_aerosimx_core, m) {
  m.doc() = "AeroSimX Core Simulation Engine Python Bindings";

  // Common Types
  py::class_<Vector3>(m, "Vector3")
      .def(py::init<double, double, double>())
      .def_readwrite("x", &Vector3::x)
      .def_readwrite("y", &Vector3::y)
      .def_readwrite("z", &Vector3::z);

  py::class_<Quaternion>(m, "Quaternion")
      .def(py::init<double, double, double, double>())
      .def_readwrite("w", &Quaternion::w)
      .def_readwrite("x", &Quaternion::x)
      .def_readwrite("y", &Quaternion::y)
      .def_readwrite("z", &Quaternion::z);

  py::class_<Pose>(m, "Pose")
      .def(py::init<>())
      .def_readwrite("position", &Pose::position)
      .def_readwrite("orientation", &Pose::orientation);

  // Simulation
  py::class_<core::Simulation>(m, "Simulation")
      .def(py::init<>())
      .def("initialize", &core::Simulation::initialize)
      .def("start", &core::Simulation::start)
      .def("stop", &core::Simulation::stop)
      .def("step", &core::Simulation::step)
      .def("reset", &core::Simulation::reset)
      .def("get_simulation_time", &core::Simulation::get_simulation_time);

  // Sensors
  py::class_<sensors::SensorBase, std::shared_ptr<sensors::SensorBase>>(
      m, "SensorBase")
      .def("get_name", &sensors::SensorBase::get_name)
      .def("is_enabled", &sensors::SensorBase::is_enabled)
      .def("set_enabled", &sensors::SensorBase::set_enabled);

  py::class_<sensors::Lidar, sensors::SensorBase,
             std::shared_ptr<sensors::Lidar>>(m, "Lidar")
      .def(py::init<const sensors::LidarConfig &>())
      .def("get_point_cloud", &sensors::Lidar::get_point_cloud);

  // Vehicles
  py::class_<vehicles::VehicleBase, std::shared_ptr<vehicles::VehicleBase>>(
      m, "VehicleBase")
      .def("get_name", &vehicles::VehicleBase::get_name)
      .def("get_state", &vehicles::VehicleBase::get_state)
      .def("attach_sensor", &vehicles::VehicleBase::attach_sensor);

  py::class_<vehicles::Multirotor, vehicles::VehicleBase,
             std::shared_ptr<vehicles::Multirotor>>(m, "Multirotor")
      .def(py::init<const vehicles::MultirotorConfig &>())
      .def("arm", &vehicles::Multirotor::arm)
      .def("disarm", &vehicles::Multirotor::disarm)
      .def("takeoff", &vehicles::Multirotor::takeoff)
      .def("land", &vehicles::Multirotor::land)
      .def("move_to_position", &vehicles::Multirotor::move_to_position);
}
