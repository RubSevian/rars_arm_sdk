#include "include/arm_motor_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rars_arm
{

ArmMotorControl::ArmMotorControl()
    : ArmMotorControl({ArmMotorType::DM4340,
                       ArmMotorType::DM4340,
                       ArmMotorType::DM4340,
                       ArmMotorType::DM4310,
                       ArmMotorType::DM4310,
                       ArmMotorType::DM4310,
                       ArmMotorType::DM4310})
{}

ArmMotorControl::ArmMotorControl(std::array<ArmMotorType, kArmMotorCount> motor_types)
    : dm4310_protocol_{
          -12.5F,
          12.5F,
          -30.0F,
          30.0F,
          0.0F,
          500.0F,
          0.0F,
          5.0F,
          -10.0F,
          10.0F},
      dm4340_protocol_{
          -12.5F,
          12.5F,
          -10.0F,
          10.0F,
          0.0F,
          500.0F,
          0.0F,
          5.0F,
          -28.0F,
          28.0F},
      safety_limits_{{
          {-1.5F, 1.5F, -2.57F, 2.57F, -15.0F, 15.0F},
          {-1.3F, 1.5F, -2.57F, 2.57F, -15.0F, 15.0F},
          {-1.5F, 1.5F, -2.57F, 2.57F, -15.0F, 15.0F},

          {-1.5F, 1.5F, -4.57F, 4.57F, -7.0F, 7.0F},
          {-1.5F, 1.5F, -4.57F, 4.57F, -7.0F, 7.0F},
          {-1.57F, 1.57F, -4.57F, 4.57F, -5.0F, 5.0F},
          {-1.57F, 1.57F, -4.57F, 4.57F, -5.0F, 5.0F}
      }},
      configured_motor_types_(std::move(motor_types)),
      last_command_motor_types_(configured_motor_types_)
{}

bool ArmMotorControl::enable(ArmSerialPort& serial)
{
    ArmSerialPort::Payload payload{};

    const auto special_command = makeSpecialCommand(EnableByte);

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        std::copy(special_command.begin(),
                  special_command.end(),
                  payload.begin() + static_cast<std::ptrdiff_t>(i * 8));
    }

    return serial.sendPayload(payload);
}

bool ArmMotorControl::disable(ArmSerialPort& serial)
{
    ArmSerialPort::Payload payload{};

    const auto special_command = makeSpecialCommand(DisableByte);

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        std::copy(special_command.begin(),
                  special_command.end(),
                  payload.begin() + static_cast<std::ptrdiff_t>(i * 8));
    }

    return serial.sendPayload(payload);
}

bool ArmMotorControl::setZero(ArmSerialPort& serial)
{
    ArmSerialPort::Payload payload{};

    const auto special_command = makeSpecialCommand(SetZeroByte);

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        std::copy(special_command.begin(),
                  special_command.end(),
                  payload.begin() + static_cast<std::ptrdiff_t>(i * 8));
    }

    return serial.sendPayload(payload);
}

bool ArmMotorControl::write(ArmSerialPort& serial, const ArmLowCmd& low_cmd)
{
    ArmSerialPort::Payload payload{};

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        const auto& command = low_cmd.motor_cmd()[i];
        if (command.mode() != ArmControlMode::MIT)
        {
            setLastError("Для мотора поддерживается только MIT mode.");
            return false;
        }

        if (!validateMotorType(i, command.motor_type()))
            return false;

        const auto frame = packMitCommand(command, i);

        std::copy(frame.begin(), frame.end(), payload.begin() + static_cast<std::ptrdiff_t>(i * 8));

        last_command_motor_types_[i] = command.motor_type();
    }

    return serial.sendPayload(payload);
}

bool ArmMotorControl::read(ArmSerialPort& serial, ArmLowState& low_state)
{
    ArmSerialPort::Payload payload{};

    if (!serial.getReceivedPayload(payload))
        return false;

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
        if (!decodeMotorFeedback(payload,
                                 i,
                                 last_command_motor_types_[i],
                                 low_state.motor_state_[i]))
            low_state.motor_state_[i].valid_ = false;

    return true;
}

std::array<std::uint8_t, 8> ArmMotorControl::packMitCommand(const ArmMotorCmd& command,
                                                            std::size_t		   motor_index)
{
    const auto& protocol = protocolLimits(command.motor_type());

    const auto& safety = safety_limits_[motor_index];

    const float q = std::clamp(command.q(), safety.position_min, safety.position_max);

    const float dq = std::clamp(command.dq(), safety.velocity_min, safety.velocity_max);

    const float kp = std::clamp(command.kp(), protocol.kp_min, protocol.kp_max);

    const float kd = std::clamp(command.kd(), protocol.kd_min, protocol.kd_max);

    const float tau = std::clamp(command.tau(), safety.torque_min, safety.torque_max);

    const std::uint32_t q_int = floatToUint(q, protocol.position_min, protocol.position_max, 16);

    const std::uint32_t dq_int = floatToUint(dq, protocol.velocity_min, protocol.velocity_max, 12);

    const std::uint32_t kp_int = floatToUint(kp, protocol.kp_min, protocol.kp_max, 12);

    const std::uint32_t kd_int = floatToUint(kd, protocol.kd_min, protocol.kd_max, 12);

    const std::uint32_t tau_int = floatToUint(tau, protocol.torque_min, protocol.torque_max, 12);

    std::array<std::uint8_t, 8> frame{};

    frame[0] = static_cast<std::uint8_t>(q_int >> 8U);

    frame[1] = static_cast<std::uint8_t>(q_int & 0xFFU);

    frame[2] = static_cast<std::uint8_t>(dq_int >> 4U);

    frame[3] = static_cast<std::uint8_t>(((dq_int & 0x0FU) << 4U) | (kp_int >> 8U));

    frame[4] = static_cast<std::uint8_t>(kp_int & 0xFFU);

    frame[5] = static_cast<std::uint8_t>(kd_int >> 4U);

    frame[6] = static_cast<std::uint8_t>(((kd_int & 0x0FU) << 4U) | (tau_int >> 8U));

    frame[7] = static_cast<std::uint8_t>(tau_int & 0xFFU);

    return frame;
}

std::array<std::uint8_t, 8> ArmMotorControl::makeSpecialCommand(std::uint8_t special_byte) const
{
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, special_byte};
}

bool ArmMotorControl::decodeMotorFeedback(const ArmSerialPort::Payload& payload,
                                          std::size_t					motor_index,
                                          ArmMotorType					motor_type,
                                          ArmMotorState&				state)
{
    if (motor_index >= kArmMotorCount)
        return false;

    if (motor_type == ArmMotorType::Unknown)
        return false;

    const auto& protocol = protocolLimits(motor_type);

    const std::size_t offset = motor_index * 8;

    const std::uint8_t status = payload[offset];

    const std::uint32_t q_int = (static_cast<std::uint32_t>(payload[offset + 1]) << 8U)
                              | static_cast<std::uint32_t>(payload[offset + 2]);

    const std::uint32_t dq_int = (static_cast<std::uint32_t>(payload[offset + 3]) << 4U)
                               | (static_cast<std::uint32_t>(payload[offset + 4]) >> 4U);

    const std::uint32_t tau_int = (static_cast<std::uint32_t>(payload[offset + 4] & 0x0FU) << 8U)
                                | static_cast<std::uint32_t>(payload[offset + 5]);

    state.motor_id_ = static_cast<std::uint8_t>(status & 0x0FU);

    state.error_ = static_cast<std::uint8_t>((status >> 4U) & 0x0FU);

    state.q_ = uintToFloat(q_int, protocol.position_min, protocol.position_max, 16);

    state.dq_ = uintToFloat(dq_int, protocol.velocity_min, protocol.velocity_max, 12);

    state.tau_ = uintToFloat(tau_int, protocol.torque_min, protocol.torque_max, 12);

    state.mos_temperature_ = static_cast<float>(payload[offset + 6]);

    state.rotor_temperature_ = static_cast<float>(payload[offset + 7]);

    const std::uint8_t expected_id = static_cast<std::uint8_t>(motor_index + 1U);

    state.valid_ = state.motor_id_ == expected_id;

    return state.valid_;

    // state.valid_ = true;

    return true;
}

const ArmMotorControl::MotorProtocolLimits& ArmMotorControl::protocolLimits(
  ArmMotorType motor_type) const
{
    switch (motor_type)
    {
        case ArmMotorType::DM4310:
            return dm4310_protocol_;

        case ArmMotorType::DM4340:
            return dm4340_protocol_;

        case ArmMotorType::Unknown:
            throw std::invalid_argument("Не задан тип двигателя.");
    }

    throw std::invalid_argument("Неизвестный тип двигателя.");
}

bool ArmMotorControl::validateMotorType(std::size_t motor_index, ArmMotorType motor_type)
{
    if (motor_index >= kArmMotorCount)
    {
        setLastError("Недопустимый индекс мотора.");
        return false;
    }

    if (motor_type == ArmMotorType::Unknown)
    {
        setLastError("Тип двигателя не задан.");
        return false;
    }

    if (motor_type != configured_motor_types_[motor_index])
    {
        setLastError("Тип двигателя не совпадает с конфигурацией руки.");
        return false;
    }

    return true;
}

std::uint32_t ArmMotorControl::floatToUint(float		value,
                                           float		value_min,
                                           float		value_max,
                                           unsigned int bits)
{
    value = std::clamp(value, value_min, value_max);

    const float span = value_max - value_min;

    const std::uint32_t maximum_integer = (1U << bits) - 1U;

    const float normalized = (value - value_min) / span;

    return static_cast<std::uint32_t>(normalized * static_cast<float>(maximum_integer));
}

float ArmMotorControl::uintToFloat(std::uint32_t value,
                                   float		 value_min,
                                   float		 value_max,
                                   unsigned int	 bits)
{
    const std::uint32_t maximum_integer = (1U << bits) - 1U;

    const float span = value_max - value_min;

    return static_cast<float>(value) * span / static_cast<float>(maximum_integer) + value_min;
}

void ArmMotorControl::setLastError(const std::string& message)
{
    last_error_ = message;
}

std::string ArmMotorControl::lastError() const
{
    return last_error_;
}

} // namespace rars_arm
