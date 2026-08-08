#include "rars_arm.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace rars_arm
{

RarsArm::RarsArm(ArmConfiguration configuration)
    : configuration_(std::move(configuration)),
      serial_(configuration_.port_name, configuration_.baud_rate),
      motor_control_(motorTypes(configuration_))
{
    validateConfiguration(configuration_);
}

RarsArm::~RarsArm()
{
    disconnect();
}

bool RarsArm::connect()
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (serial_.isOpen() && serial_.isReceiving())
        return true;

    if (serial_.isOpen())
        serial_.close();

    if (!serial_.open())
    {
        copySerialError("Failed to connect");
        return false;
    }

    if (!serial_.startReceiving())
    {
        copySerialError("Failed to start feedback receiver");
        serial_.close();
        return false;
    }

    enabled_ = false;
    watchdog_armed_ = false;
    setLastError("");
    return true;
}

void RarsArm::disconnect() noexcept
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (enabled_ && serial_.isOpen())
    {
        // Best effort: destruction and shutdown must never throw or hang on an
        // error-reporting path.
        static_cast<void>(motor_control_.disable(serial_, configuration_.control_modes));
    }

    enabled_ = false;
    watchdog_armed_ = false;
    serial_.close();
}

bool RarsArm::isConnected() const noexcept
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    return serial_.isOpen() && serial_.isReceiving();
}

bool RarsArm::isEnabled() const noexcept
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    return enabled_;
}

bool RarsArm::enable()
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (!serial_.isOpen() || !serial_.isReceiving())
    {
        setLastError("Cannot enable motors: the arm is not connected.");
        return false;
    }

    if (enabled_)
        return true;

    // Drop feedback received while the motors were disabled. Do this before
    // sending enable so that the first enabled-state replies are preserved.
    serial_.resetStatistics();

    if (!motor_control_.enable(serial_, configuration_.control_modes))
    {
        copySerialError("Failed to send enable command");
        return false;
    }

    // Preserve the proven hardware-test sequence: the controller receives the
    // enable frame twice before cyclic commands begin.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!motor_control_.enable(serial_, configuration_.control_modes))
    {
        copySerialError("Failed to confirm enable command");
        static_cast<void>(motor_control_.disable(serial_, configuration_.control_modes));
        return false;
    }

    enabled_ = true;
    watchdog_armed_ = false;
    setLastError("");
    return true;
}

bool RarsArm::disable()
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (!serial_.isOpen())
    {
        enabled_ = false;
        setLastError("Cannot disable motors: the arm is not connected.");
        return false;
    }

    if (!motor_control_.disable(serial_, configuration_.control_modes))
    {
        copySerialError("Failed to send disable command");
        return false;
    }

    enabled_ = false;
    watchdog_armed_ = false;
    setLastError("");
    return true;
}

bool RarsArm::setZero()
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (!serial_.isOpen() || !serial_.isReceiving())
    {
        setLastError("Cannot set zero: the arm is not connected.");
        return false;
    }

    if (enabled_)
    {
        setLastError("Cannot set zero while motors are enabled. Disable them first.");
        return false;
    }

    if (!motor_control_.setZero(serial_, configuration_.control_modes))
    {
        copySerialError("Failed to send set-zero command");
        return false;
    }

    setLastError("");
    return true;
}

bool RarsArm::sendMit(const MotorValues& position,
                      const MotorValues& velocity,
                      const MotorValues& kp,
                      const MotorValues& kd,
                      const MotorValues& torque)
{
    ArmMotorControl::ControlModes modes{};
    modes.fill(ArmControlMode::MIT);
    return sendWithModes(position, velocity, kp, kd, torque, modes);
}

bool RarsArm::sendConfigured(const MotorValues& position,
                             const MotorValues& velocity,
                             const MotorValues& kp,
                             const MotorValues& kd,
                             const MotorValues& torque)
{
    return sendWithModes(position,
                         velocity,
                         kp,
                         kd,
                         torque,
                         configuration_.control_modes);
}

bool RarsArm::sendWithModes(const MotorValues& position,
                            const MotorValues& velocity,
                            const MotorValues& kp,
                            const MotorValues& kd,
                            const MotorValues& torque,
                            const ArmMotorControl::ControlModes& modes)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (!serial_.isOpen() || !serial_.isReceiving())
    {
        setLastError("Cannot send a command: the arm is not connected.");
        return false;
    }

    if (!enabled_)
    {
        setLastError("Cannot send a command: motors are disabled.");
        return false;
    }

    if (modes != configuration_.control_modes)
    {
        setLastError("Command modes differ from the modes selected before enable.");
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const SerialStatistics serial_statistics = serial_.statistics();
    if (watchdog_armed_ && watchdogTripped(serial_statistics, now))
    {
        setLastError("Cannot send a command: feedback watchdog timed out.");
        return false;
    }

    if (!allFinite(position) || !allFinite(velocity) || !allFinite(kp)
        || !allFinite(kd) || !allFinite(torque))
    {
        setLastError("MIT command contains NaN or infinity.");
        return false;
    }

    std::size_t invalid_index = 0;
    if (!positionsWithinLimits(position, invalid_index))
    {
        const auto& motor = configuration_.motors[invalid_index];
        setLastError("Motor " + std::to_string(invalid_index + 1U)
                     + " joint position " + std::to_string(position[invalid_index])
                     + " is outside configured limits ["
                     + std::to_string(motor.joint_position_min) + ", "
                     + std::to_string(motor.joint_position_max) + "].");
        return false;
    }

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        if (modes[i] == ArmControlMode::MIT
            && std::abs(velocity[i]) > configuration_.motors[i].joint_velocity_max)
        {
            setLastError("Motor " + std::to_string(i + 1U)
                         + " joint velocity " + std::to_string(velocity[i])
                         + " is outside configured limits ["
                         + std::to_string(-configuration_.motors[i].joint_velocity_max)
                         + ", "
                         + std::to_string(configuration_.motors[i].joint_velocity_max)
                         + "].");
            return false;
        }
        if (modes[i] == ArmControlMode::MIT
            && std::abs(torque[i]) > configuration_.motors[i].joint_torque_max)
        {
            setLastError("Motor " + std::to_string(i + 1U)
                         + " joint torque " + std::to_string(torque[i])
                         + " is outside configured limits ["
                         + std::to_string(-configuration_.motors[i].joint_torque_max)
                         + ", "
                         + std::to_string(configuration_.motors[i].joint_torque_max)
                         + "].");
            return false;
        }
    }

    ArmLowCmd command;
    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        auto& motor = command.motor_cmd()[i];
        const auto& motor_configuration = configuration_.motors[i];
        motor.motor_type() = motor_configuration.type;
        motor.mode() = modes[i];
        motor.q() = motor_configuration.direction * position[i]
                    + motor_configuration.zero_offset;
        motor.dq() = modes[i] == ArmControlMode::PositionVelocity
                       ? configuration_.position_velocity_limits[i]
                       : motor_configuration.direction * velocity[i];
        motor.kp() = kp[i];
        motor.kd() = kd[i];
        motor.tau() = motor_configuration.direction * torque[i];
    }

    if (!motor_control_.write(serial_, command))
    {
        const std::string protocol_error = motor_control_.lastError();
        if (!protocol_error.empty())
            copyMotorError("Failed to send MIT command");
        else
            copySerialError("Failed to send MIT command");
        return false;
    }

    if (!watchdog_armed_)
    {
        watchdog_armed_ = true;
        watchdog_armed_at_ = now;
    }

    setLastError("");
    return true;
}

bool RarsArm::sendPositionTargets(const MotorValues& position)
{
    const MotorValues zeros{};
    return sendConfigured(position,
                          zeros,
                          configuration_.default_kp,
                          configuration_.default_kd,
                          zeros);
}

bool RarsArm::setControlModes(const ArmMotorControl::ControlModes& modes)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (enabled_)
    {
        setLastError("Cannot change control modes while motors are enabled.");
        return false;
    }
    configuration_.control_modes = modes;
    setLastError("");
    return true;
}

ArmMotorControl::ControlModes RarsArm::controlModes() const
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    return configuration_.control_modes;
}

bool RarsArm::tryReadState(ArmLowState& state)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    if (!serial_.isOpen() || !serial_.isReceiving())
    {
        setLastError("Cannot read state: the arm is not connected.");
        return false;
    }

    return motor_control_.read(serial_, state);
}

bool RarsArm::tryReadJointState(JointState& state)
{
    ArmLowState raw_state;
    if (!tryReadState(raw_state))
        return false;

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        const auto& raw_motor = raw_state.motor_state()[i];
        const auto& motor_configuration = configuration_.motors[i];
        state.position[i] = motor_configuration.direction
                            * (raw_motor.q() - motor_configuration.zero_offset);
        state.velocity[i] = motor_configuration.direction * raw_motor.dq();
        state.torque[i] = motor_configuration.direction * raw_motor.tau();
        state.mos_temperature[i] = raw_motor.mos_temperature();
        state.rotor_temperature[i] = raw_motor.rotor_temperature();
        state.error[i] = raw_motor.error();
        state.motor_id[i] = raw_motor.motor_id();
        state.valid[i] = raw_motor.valid();
    }

    return true;
}

CommunicationStatus RarsArm::communicationStatus() const
{
    std::lock_guard<std::mutex> lock(operation_mutex_);

    const SerialStatistics serial_status = serial_.statistics();
    CommunicationStatus status;
    status.connected = serial_.isOpen() && serial_.isReceiving();
    status.enabled = enabled_;
    status.feedback_received = serial_status.feedback_received;
    status.watchdog_armed = watchdog_armed_;
    status.watchdog_tripped = status.connected && watchdog_armed_
                              && watchdogTripped(serial_status, std::chrono::steady_clock::now());
    status.protocol_v2_detected = motor_control_.protocolV2Detected();
    status.stm32_watchdog_tripped = motor_control_.stm32WatchdogTripped();
    status.last_acknowledged_control = motor_control_.lastAcknowledgedControl();
    if (status.protocol_v2_detected && serial_status.feedback_received)
        status.board_temperature_c = static_cast<int>(serial_.lastReceivedPayload()[58] >> 2U) * 2;
    status.valid_frames = serial_status.valid_frames;
    status.invalid_frames = serial_status.invalid_frames;
    status.read_timeouts = serial_status.read_timeouts;
    status.read_errors = serial_status.read_errors;

    if (serial_status.feedback_received)
    {
        status.feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - serial_status.last_valid_frame);
    }

    return status;
}

const ArmConfiguration& RarsArm::configuration() const noexcept
{
    return configuration_;
}

std::string RarsArm::lastError() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

bool RarsArm::allFinite(const MotorValues& values) noexcept
{
    for (const float value : values)
        if (!std::isfinite(value))
            return false;
    return true;
}

void RarsArm::validateConfiguration(const ArmConfiguration& configuration)
{
    if (configuration.port_name.empty())
        throw std::invalid_argument("Serial port name must not be empty.");
    if (configuration.baud_rate == 0U)
        throw std::invalid_argument("Baud rate must be greater than zero.");
    if (!std::isfinite(configuration.command_rate_hz)
        || configuration.command_rate_hz <= 0.0F
        || configuration.command_rate_hz > 500.0F)
        throw std::invalid_argument("Command rate must be in (0, 500] Hz.");
    if (configuration.feedback_timeout.count() <= 0
        || configuration.initial_feedback_grace.count() <= 0)
        throw std::invalid_argument("Feedback watchdog timeouts must be greater than zero.");

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        const auto& motor = configuration.motors[i];
        if (motor.type == ArmMotorType::Unknown)
            throw std::invalid_argument("Motor type is not configured at index "
                                        + std::to_string(i) + ".");
        if (motor.direction != 1.0F && motor.direction != -1.0F)
            throw std::invalid_argument("Motor direction must be +1 or -1 at index "
                                        + std::to_string(i) + ".");
        if (!std::isfinite(motor.zero_offset) || !std::isfinite(motor.joint_position_min)
            || !std::isfinite(motor.joint_position_max)
            || !std::isfinite(motor.joint_velocity_max)
            || !std::isfinite(motor.joint_torque_max)
            || motor.joint_position_min >= motor.joint_position_max
            || motor.joint_velocity_max <= 0.0F || motor.joint_torque_max <= 0.0F)
            throw std::invalid_argument("Invalid motor limits or offset at index "
                                        + std::to_string(i) + ".");

        if (!std::isfinite(configuration.default_kp[i])
            || configuration.default_kp[i] < 0.0F
            || configuration.default_kp[i] > 500.0F)
            throw std::invalid_argument("KP must be in [0, 500] at index "
                                        + std::to_string(i) + ".");
        if (!std::isfinite(configuration.default_kd[i])
            || configuration.default_kd[i] < 0.0F
            || configuration.default_kd[i] > 5.0F)
            throw std::invalid_argument("KD must be in [0, 5] at index "
                                        + std::to_string(i) + ".");

        if (configuration.control_modes[i] != ArmControlMode::MIT
            && configuration.control_modes[i] != ArmControlMode::PositionVelocity)
            throw std::invalid_argument("Unsupported control mode at index "
                                        + std::to_string(i) + ".");
        if (!std::isfinite(configuration.position_velocity_limits[i])
            || configuration.position_velocity_limits[i] <= 0.0F
            || configuration.position_velocity_limits[i] > motor.joint_velocity_max)
            throw std::invalid_argument(
              "POS_VEL velocity limit must be within the joint velocity limit at index "
              + std::to_string(i) + ".");

        const float protocol_velocity_max =
          motor.type == ArmMotorType::DM4340 ? 10.0F : 30.0F;
        const float protocol_torque_max =
          motor.type == ArmMotorType::DM4340 ? 28.0F : 10.0F;
        if (motor.joint_velocity_max > protocol_velocity_max
            || motor.joint_torque_max > protocol_torque_max)
            throw std::invalid_argument(
              "Joint velocity or torque limit exceeds the motor protocol at index "
              + std::to_string(i) + ".");

        const float raw_at_min =
          motor.direction * motor.joint_position_min + motor.zero_offset;
        const float raw_at_max =
          motor.direction * motor.joint_position_max + motor.zero_offset;
        if (std::min(raw_at_min, raw_at_max) < -12.5F
            || std::max(raw_at_min, raw_at_max) > 12.5F)
            throw std::invalid_argument(
              "Joint position limits and zero offset exceed the motor protocol at index "
              + std::to_string(i) + ".");
    }
}

std::array<ArmMotorType, kArmMotorCount> RarsArm::motorTypes(
  const ArmConfiguration& configuration) noexcept
{
    std::array<ArmMotorType, kArmMotorCount> result{};
    for (std::size_t i = 0; i < kArmMotorCount; ++i)
        result[i] = configuration.motors[i].type;
    return result;
}

bool RarsArm::positionsWithinLimits(const MotorValues& position,
                                    std::size_t& invalid_index) const noexcept
{
    // MIT position feedback is quantized to 16 bits. Around exact boundaries
    // (notably a zero lower limit), decoding can differ by about 0.0002 rad.
    // This tolerance only absorbs representation error; it does not replace
    // the configured mechanical limits.
    constexpr float position_limit_tolerance = 8.0e-3F;

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        if (position[i] <
              configuration_.motors[i].joint_position_min - position_limit_tolerance
            || position[i] >
              configuration_.motors[i].joint_position_max + position_limit_tolerance)
        {
            invalid_index = i;
            return false;
        }
    }
    return true;
}

bool RarsArm::watchdogTripped(const SerialStatistics& statistics,
                              std::chrono::steady_clock::time_point now) const noexcept
{
    if (!configuration_.feedback_watchdog_enabled)
        return false;

    if (statistics.feedback_received)
        return now - statistics.last_valid_frame > configuration_.feedback_timeout;

    return now - watchdog_armed_at_ > configuration_.initial_feedback_grace;
}

void RarsArm::setLastError(std::string message)
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = std::move(message);
}

void RarsArm::copySerialError(const std::string& context)
{
    const std::string detail = serial_.lastError();
    setLastError(detail.empty() ? context : context + ": " + detail);
}

void RarsArm::copyMotorError(const std::string& context)
{
    const std::string detail = motor_control_.lastError();
    setLastError(detail.empty() ? context : context + ": " + detail);
}

} // namespace rars_arm
