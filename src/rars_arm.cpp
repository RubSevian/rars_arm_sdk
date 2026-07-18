#include "include/rars_arm.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <utility>

namespace rars_arm
{

namespace
{

constexpr std::array<ArmMotorType, kArmMotorCount> kMotorTypes{
  ArmMotorType::DM4340,
  ArmMotorType::DM4340,
  ArmMotorType::DM4340,
  ArmMotorType::DM4310,
  ArmMotorType::DM4310,
  ArmMotorType::DM4310,
  ArmMotorType::DM4310};

} // namespace

RarsArm::RarsArm(ArmConfiguration configuration)
    : configuration_(std::move(configuration)),
      serial_(configuration_.port_name, configuration_.baud_rate)
{}

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
        static_cast<void>(motor_control_.disable(serial_));
    }

    enabled_ = false;
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

    if (!motor_control_.enable(serial_))
    {
        copySerialError("Failed to send enable command");
        return false;
    }

    // Preserve the proven hardware-test sequence: the controller receives the
    // enable frame twice before cyclic commands begin.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!motor_control_.enable(serial_))
    {
        copySerialError("Failed to confirm enable command");
        static_cast<void>(motor_control_.disable(serial_));
        return false;
    }

    enabled_ = true;
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

    if (!motor_control_.disable(serial_))
    {
        copySerialError("Failed to send disable command");
        return false;
    }

    enabled_ = false;
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

    if (!motor_control_.setZero(serial_))
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

    if (!allFinite(position) || !allFinite(velocity) || !allFinite(kp)
        || !allFinite(kd) || !allFinite(torque))
    {
        setLastError("MIT command contains NaN or infinity.");
        return false;
    }

    ArmLowCmd command;
    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        auto& motor = command.motor_cmd()[i];
        motor.motor_type() = kMotorTypes[i];
        motor.mode() = ArmControlMode::MIT;
        motor.q() = position[i];
        motor.dq() = velocity[i];
        motor.kp() = kp[i];
        motor.kd() = kd[i];
        motor.tau() = torque[i];
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

    setLastError("");
    return true;
}

bool RarsArm::sendPositionTargets(const MotorValues& position)
{
    const MotorValues zeros{};
    return sendMit(position,
                   zeros,
                   configuration_.default_kp,
                   configuration_.default_kd,
                   zeros);
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
