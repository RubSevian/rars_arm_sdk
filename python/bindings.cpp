#include "include/rars_arm.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>

namespace py = pybind11;

PYBIND11_MODULE(rars_arm_py, module)
{
    using namespace rars_arm;

    module.doc() = "Python bindings for the RARS arm SDK";
    module.attr("ARM_JOINT_COUNT") = py::int_(kArmJointCount);
    module.attr("GRIPPER_MOTOR_INDEX") = py::int_(kGripperMotorIndex);
    module.attr("MOTOR_COUNT") = py::int_(kArmMotorCount);

    py::enum_<ArmMotorType>(module, "ArmMotorType")
      .value("UNKNOWN", ArmMotorType::Unknown)
      .value("DM4310", ArmMotorType::DM4310)
      .value("DM4340", ArmMotorType::DM4340);

    py::class_<MotorConfiguration>(module, "MotorConfiguration")
      .def(py::init<>())
      .def_readwrite("type", &MotorConfiguration::type)
      .def_readwrite("direction", &MotorConfiguration::direction)
      .def_readwrite("zero_offset", &MotorConfiguration::zero_offset)
      .def_readwrite("joint_position_min", &MotorConfiguration::joint_position_min)
      .def_readwrite("joint_position_max", &MotorConfiguration::joint_position_max)
      .def_readwrite("joint_velocity_max", &MotorConfiguration::joint_velocity_max)
      .def_readwrite("joint_torque_max", &MotorConfiguration::joint_torque_max);

    py::class_<ArmConfiguration>(module, "ArmConfiguration")
      .def(py::init<>())
      .def_readwrite("port_name", &ArmConfiguration::port_name)
      .def_readwrite("baud_rate", &ArmConfiguration::baud_rate)
      .def_readwrite("default_kp", &ArmConfiguration::default_kp)
      .def_readwrite("default_kd", &ArmConfiguration::default_kd)
      .def_readwrite("motors", &ArmConfiguration::motors)
      .def(
        "motor",
        [](ArmConfiguration& configuration, std::size_t index)
          -> MotorConfiguration& {
            if (index >= kArmMotorCount)
                throw py::index_error("motor index is out of range");
            return configuration.motors[index];
        },
        py::arg("index"),
        py::return_value_policy::reference_internal)
      .def_readwrite("feedback_watchdog_enabled",
                     &ArmConfiguration::feedback_watchdog_enabled)
      .def_property(
        "feedback_timeout_ms",
        [](const ArmConfiguration& configuration) {
            return configuration.feedback_timeout.count();
        },
        [](ArmConfiguration& configuration, std::int64_t value) {
            configuration.feedback_timeout = std::chrono::milliseconds(value);
        })
      .def_property(
        "initial_feedback_grace_ms",
        [](const ArmConfiguration& configuration) {
            return configuration.initial_feedback_grace.count();
        },
        [](ArmConfiguration& configuration, std::int64_t value) {
            configuration.initial_feedback_grace = std::chrono::milliseconds(value);
        });

    py::class_<CommunicationStatus>(module, "CommunicationStatus")
      .def_readonly("connected", &CommunicationStatus::connected)
      .def_readonly("enabled", &CommunicationStatus::enabled)
      .def_readonly("feedback_received", &CommunicationStatus::feedback_received)
      .def_readonly("watchdog_armed", &CommunicationStatus::watchdog_armed)
      .def_readonly("watchdog_tripped", &CommunicationStatus::watchdog_tripped)
      .def_property_readonly("feedback_age_ms", [](const CommunicationStatus& status) {
          return status.feedback_age.count();
      })
      .def_readonly("valid_frames", &CommunicationStatus::valid_frames)
      .def_readonly("invalid_frames", &CommunicationStatus::invalid_frames)
      .def_readonly("read_timeouts", &CommunicationStatus::read_timeouts)
      .def_readonly("read_errors", &CommunicationStatus::read_errors);

    py::class_<JointState>(module, "JointState")
      .def_readonly("position", &JointState::position)
      .def_readonly("velocity", &JointState::velocity)
      .def_readonly("torque", &JointState::torque)
      .def_readonly("mos_temperature", &JointState::mos_temperature)
      .def_readonly("rotor_temperature", &JointState::rotor_temperature)
      .def_readonly("error", &JointState::error)
      .def_readonly("motor_id", &JointState::motor_id)
      .def_readonly("valid", &JointState::valid);

    py::class_<RarsArm>(module, "RarsArm")
      .def(py::init<ArmConfiguration>(), py::arg("configuration") = ArmConfiguration{})
      .def("connect", &RarsArm::connect, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &RarsArm::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("is_connected", &RarsArm::isConnected)
      .def("is_enabled", &RarsArm::isEnabled)
      .def("enable", &RarsArm::enable, py::call_guard<py::gil_scoped_release>())
      .def("disable", &RarsArm::disable, py::call_guard<py::gil_scoped_release>())
      .def("set_zero", &RarsArm::setZero, py::call_guard<py::gil_scoped_release>())
      .def(
        "send_mit",
        &RarsArm::sendMit,
        py::arg("position"),
        py::arg("velocity"),
        py::arg("kp"),
        py::arg("kd"),
        py::arg("torque"),
        py::call_guard<py::gil_scoped_release>())
      .def("send_position_targets",
           &RarsArm::sendPositionTargets,
           py::arg("position"),
           py::call_guard<py::gil_scoped_release>())
      .def("try_read_joint_state", [](RarsArm& arm) -> py::object {
          JointState state;
          bool received = false;
          {
              py::gil_scoped_release release;
              received = arm.tryReadJointState(state);
          }
          return received ? py::cast(state) : py::none();
      })
      .def("communication_status", &RarsArm::communicationStatus)
      .def_property_readonly("configuration", [](const RarsArm& arm) {
          return arm.configuration();
      })
      .def_property_readonly("last_error", &RarsArm::lastError);
}
