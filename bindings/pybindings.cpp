#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motionSystem.hpp"
#include "motors.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace py = pybind11;

namespace {
void set_callback_py(CyclicSession& session, py::function fn) {
  session.setCallback([fn = std::move(fn)](AllMotors&, bool& shutdown) {
    py::gil_scoped_acquire gil;
    py::object result = fn();
    if (!result.is_none() && static_cast<bool>(py::bool_(result))) {
      shutdown = true;
    }
  });
}
} // namespace

PYBIND11_MODULE(motorcode, m) {
  m.doc() = "MotorCode high-level API bindings";

  py::enum_<MotorModes>(m, "MotorModes")
      .value("PP_Mode", PP_Mode)
      .value("CSP_Mode", CSP_Mode)
      .value("CSV_Mode", CSV_Mode)
      .value("CST_Mode", CST_Mode)
      .value("UNKNOWN", UNKNOWN);

  py::enum_<MotorModel>(m, "MotorModel")
      .value("PanasonicA6MotorType", PanasonicA6MotorType)
      .value("FakeMotorType", FakeMotorType)
      .value("UnknownMotorType", UnknownMotorType);

  py::class_<MotionProfile>(m, "MotionProfile")
      .def(py::init<>())
      .def_readwrite("vel", &MotionProfile::vel)
      .def_readwrite("acc", &MotionProfile::acc)
      .def_readwrite("dec", &MotionProfile::dec);

  py::class_<Motor, std::unique_ptr<Motor, py::nodelete>>(m, "Motor")
      .def("set_mode", &Motor::set_mode)
      .def("set_target_position", &Motor::set_target_position)
      .def("set_target_velocity", &Motor::set_target_velocity)
      .def("set_target_torque", &Motor::set_target_torque)
      .def("setMotionProfile", &Motor::setMotionProfile)
      .def("setVel", &Motor::setVel)
      .def("setAcc", &Motor::setAcc)
      .def("setDec", &Motor::setDec)
      .def("servo_on", &Motor::servo_on)
      .def("servo_off", &Motor::servo_off)
      .def("get_error_code", &Motor::get_error_code)
      .def("get_current_position", &Motor::get_current_position)
      .def("get_current_velocity", &Motor::get_current_velocity)
      .def("get_current_torque", &Motor::get_current_torque)
      .def("get_mode", &Motor::get_mode);

  py::class_<MotionSystem>(m, "MotionSystem")
      .def(py::init<>())
      .def(py::init<MotorModel, const char*>())
      .def("start_connect", &MotionSystem::start_connect, py::arg("motor_count"),
           py::arg("cyclePeriod"), py::arg("mode") = PP_Mode)
      .def("motor", [](MotionSystem& self, int id) -> Motor& {
        return self.motors().motor(id);
      }, py::return_value_policy::reference_internal, py::arg("id"))
      .def("motor_count", [](MotionSystem& self) {
        return self.motors().count();
      })
      .def("set_callback", [](MotionSystem& self, py::function fn) {
        set_callback_py(self.session(), std::move(fn));
      }, py::arg("callback"))
      .def("run_async", &MotionSystem::run_async)
      .def("run_async_io_only", &MotionSystem::run_async_io_only)
      .def("get_slave_count", [](MotionSystem& self) {
        int count = 0;
        if (!self.get_slave_count(count)) {
          throw std::runtime_error("get_slave_count failed");
        }
        return count;
      })
      .def("start_control_loop", &MotionSystem::start_control_loop)
      .def("set_cycle_log_enabled", &MotionSystem::set_cycle_log_enabled)
      .def("stop", &MotionSystem::stop)
      .def("close", &MotionSystem::close);
}
