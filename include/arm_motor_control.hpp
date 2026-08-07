#pragma once

#include "arm_serial_port.hpp"
#include "arm_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace rars_arm
{

class ArmMotorControl
{
public:

    ArmMotorControl();
    explicit ArmMotorControl(std::array<ArmMotorType, kArmMotorCount> motor_types);

    using ControlModes = std::array<ArmControlMode, kArmMotorCount>;

    bool enable(ArmSerialPort& serial, const ControlModes& modes);

    bool disable(ArmSerialPort& serial, const ControlModes& modes);

    bool setZero(ArmSerialPort& serial, const ControlModes& modes);

    bool write(ArmSerialPort& serial, const ArmLowCmd& command);

    bool read(ArmSerialPort& serial, ArmLowState& state);

    [[nodiscard]] bool protocolV2Detected() const noexcept;
    [[nodiscard]] bool stm32WatchdogTripped() const noexcept;
    [[nodiscard]] std::uint8_t lastAcknowledgedControl() const noexcept;

    [[nodiscard]] std::string lastError() const;

private:

    struct MotorProtocolLimits
    {
        float position_min;
        float position_max;

        float velocity_min;
        float velocity_max;

        float kp_min;
        float kp_max;

        float kd_min;
        float kd_max;

        float torque_min;
        float torque_max;
    };

private:

    [[nodiscard]] std::array<std::uint8_t, 8> packMitCommand(const ArmMotorCmd& command);

    [[nodiscard]] std::array<std::uint8_t, 8> packPositionVelocityCommand(
      const ArmMotorCmd& command) const;

    [[nodiscard]] std::array<std::uint8_t, 8> makeSpecialCommand(std::uint8_t special_byte) const;

    bool sendAction(ArmSerialPort& serial,
                    const ControlModes& modes,
                    std::uint8_t action);

    void addProtocolMetadata(ArmSerialPort::Payload& payload,
                             const ControlModes& modes,
                             std::uint8_t action);

    [[nodiscard]] static std::uint8_t modeMask(const ControlModes& modes);

    bool decodeMotorFeedback(const ArmSerialPort::Payload& payload,
                             std::size_t				   motor_index,
                             ArmMotorType				   motor_type,
                             ArmMotorState&				   state);

    [[nodiscard]] const MotorProtocolLimits& protocolLimits(ArmMotorType motor_type) const;

    bool validateMotorType(std::size_t motor_index, ArmMotorType motor_type);
    bool validateProtocolLimits(const ArmMotorCmd& command, std::size_t motor_index);

    static std::uint32_t floatToUint(float		  value,
                                     float		  value_min,
                                     float		  value_max,
                                     unsigned int bits);

    static float uintToFloat(std::uint32_t value,
                             float		   value_min,
                             float		   value_max,
                             unsigned int  bits);

    void setLastError(const std::string& message);

private:

    static constexpr std::uint8_t EnableByte  = 0xFC;
    static constexpr std::uint8_t DisableByte = 0xFD;
    static constexpr std::uint8_t SetZeroByte = 0xFE;
    static constexpr std::uint8_t ProtocolMarker = 0xA2;
    static constexpr std::size_t ProtocolMarkerIndex = 56;
    static constexpr std::size_t ProtocolModeMaskIndex = 57;
    static constexpr std::size_t ProtocolControlIndex = 58;
    static constexpr std::uint8_t ActionCommand = 0;
    static constexpr std::uint8_t ActionEnable = 1;
    static constexpr std::uint8_t ActionDisable = 2;
    static constexpr std::uint8_t ActionSetZero = 3;

    const MotorProtocolLimits dm4310_protocol_;
    const MotorProtocolLimits dm4340_protocol_;

    std::array<ArmMotorType, kArmMotorCount> configured_motor_types_;

    std::array<ArmMotorType, kArmMotorCount> last_command_motor_types_;

    std::uint8_t sequence_ = 0;
    bool protocol_v2_detected_ = false;
    bool stm32_watchdog_tripped_ = false;
    std::uint8_t last_acknowledged_control_ = 0;

    std::string last_error_;
};

} // namespace rars_arm
