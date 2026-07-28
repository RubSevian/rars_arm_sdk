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

    bool enable(ArmSerialPort& serial);

    bool disable(ArmSerialPort& serial);

    bool setZero(ArmSerialPort& serial);

    bool write(ArmSerialPort& serial, const ArmLowCmd& command);

    bool read(ArmSerialPort& serial, ArmLowState& state);

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

    struct MotorSafetyLimits
    {
        float position_min;
        float position_max;

        float velocity_min;
        float velocity_max;

        float torque_min;
        float torque_max;
    };

private:

    [[nodiscard]] std::array<std::uint8_t, 8> packMitCommand(const ArmMotorCmd& command,
                                                             std::size_t		motor_index);

    [[nodiscard]] std::array<std::uint8_t, 8> makeSpecialCommand(std::uint8_t special_byte) const;

    bool decodeMotorFeedback(const ArmSerialPort::Payload& payload,
                             std::size_t				   motor_index,
                             ArmMotorType				   motor_type,
                             ArmMotorState&				   state);

    [[nodiscard]] const MotorProtocolLimits& protocolLimits(ArmMotorType motor_type) const;

    bool validateMotorType(std::size_t motor_index, ArmMotorType motor_type);

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

    const MotorProtocolLimits dm4310_protocol_;
    const MotorProtocolLimits dm4340_protocol_;

    std::array<MotorSafetyLimits, kArmMotorCount> safety_limits_;

    std::array<ArmMotorType, kArmMotorCount> configured_motor_types_;

    std::array<ArmMotorType, kArmMotorCount> last_command_motor_types_;

    std::string last_error_;
};

} // namespace rars_arm
