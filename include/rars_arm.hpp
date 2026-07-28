#pragma once

#include "arm_motor_control.hpp"
#include "arm_serial_port.hpp"
#include "arm_types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace rars_arm
{

struct MotorConfiguration
{
    ArmMotorType type = ArmMotorType::Unknown;
    float direction = 1.0F;
    float zero_offset = 0.0F;
    float joint_position_min = -1.5F;
    float joint_position_max = 1.5F;
};

struct ArmConfiguration
{
    std::string port_name = "/dev/ttyACM0";
    unsigned int baud_rate = 921600;

    std::array<float, kArmMotorCount> default_kp{
      60.0F, 60.0F, 60.0F, 20.0F, 20.0F, 20.0F, 20.0F};
    std::array<float, kArmMotorCount> default_kd{
      1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F};

    // Entries 0..5 are arm joints; entry 6 is the gripper motor.
    std::array<MotorConfiguration, kArmMotorCount> motors{{
      {ArmMotorType::DM4340, 1.0F, 0.0F, -1.5F, 1.5F},
      {ArmMotorType::DM4340, 1.0F, 0.0F, -1.3F, 1.5F},
      {ArmMotorType::DM4340, 1.0F, 0.0F, -1.5F, 1.5F},
      {ArmMotorType::DM4310, 1.0F, 0.0F, -1.5F, 1.5F},
      {ArmMotorType::DM4310, 1.0F, 0.0F, -1.5F, 1.5F},
      {ArmMotorType::DM4310, 1.0F, 0.0F, -1.57F, 1.57F},
      {ArmMotorType::DM4310, 1.0F, 0.0F, -1.57F, 1.57F},
    }};

    bool feedback_watchdog_enabled = true;
    std::chrono::milliseconds feedback_timeout{200};
    std::chrono::milliseconds initial_feedback_grace{1000};
};

struct JointState
{
    std::array<float, kArmMotorCount> position{};
    std::array<float, kArmMotorCount> velocity{};
    std::array<float, kArmMotorCount> torque{};
    std::array<float, kArmMotorCount> mos_temperature{};
    std::array<float, kArmMotorCount> rotor_temperature{};
    std::array<std::uint8_t, kArmMotorCount> error{};
    std::array<std::uint8_t, kArmMotorCount> motor_id{};
    std::array<bool, kArmMotorCount> valid{};
};

struct CommunicationStatus
{
    bool connected = false;
    bool enabled = false;
    bool feedback_received = false;
    bool watchdog_armed = false;
    bool watchdog_tripped = false;
    std::chrono::milliseconds feedback_age{0};
    std::uint64_t valid_frames = 0;
    std::uint64_t invalid_frames = 0;
    std::uint64_t read_timeouts = 0;
    std::uint64_t read_errors = 0;
};

class RarsArm
{
public:
    using MotorValues = std::array<float, kArmMotorCount>;

    explicit RarsArm(ArmConfiguration configuration = {});
    ~RarsArm();

    RarsArm(const RarsArm&) = delete;
    RarsArm& operator=(const RarsArm&) = delete;
    RarsArm(RarsArm&&) = delete;
    RarsArm& operator=(RarsArm&&) = delete;

    bool connect();
    void disconnect() noexcept;

    [[nodiscard]] bool isConnected() const noexcept;
    [[nodiscard]] bool isEnabled() const noexcept;

    bool enable();
    bool disable();

    // Setting zero is intentionally allowed only while the motors are disabled.
    bool setZero();

    bool sendMit(const MotorValues& position,
                 const MotorValues& velocity,
                 const MotorValues& kp,
                 const MotorValues& kd,
                 const MotorValues& torque);

    bool sendPositionTargets(const MotorValues& position);

    // Returns true only when a new feedback packet has arrived since the
    // previous call. The supplied state is left unchanged otherwise.
    bool tryReadState(ArmLowState& state);

    // Feedback transformed to joint coordinates. Index 6 is the gripper.
    bool tryReadJointState(JointState& state);

    [[nodiscard]] CommunicationStatus communicationStatus() const;

    [[nodiscard]] const ArmConfiguration& configuration() const noexcept;
    [[nodiscard]] std::string lastError() const;

private:
    static void validateConfiguration(const ArmConfiguration& configuration);
    [[nodiscard]] static std::array<ArmMotorType, kArmMotorCount> motorTypes(
      const ArmConfiguration& configuration) noexcept;
    [[nodiscard]] static bool allFinite(const MotorValues& values) noexcept;
    [[nodiscard]] bool positionsWithinLimits(const MotorValues& position,
                                             std::size_t& invalid_index) const noexcept;
    [[nodiscard]] bool watchdogTripped(const SerialStatistics& statistics,
                                       std::chrono::steady_clock::time_point now) const noexcept;
    void setLastError(std::string message);
    void copySerialError(const std::string& context);
    void copyMotorError(const std::string& context);

private:
    ArmConfiguration configuration_;
    ArmSerialPort serial_;
    ArmMotorControl motor_control_;

    mutable std::mutex operation_mutex_;
    mutable std::mutex error_mutex_;
    bool enabled_ = false;
    bool watchdog_armed_ = false;
    std::chrono::steady_clock::time_point watchdog_armed_at_{};
    std::string last_error_;
};

} // namespace rars_arm
