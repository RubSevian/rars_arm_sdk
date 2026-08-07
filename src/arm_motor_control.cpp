#include "arm_motor_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstring>
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
      configured_motor_types_(std::move(motor_types)),
      last_command_motor_types_(configured_motor_types_)
{}

bool ArmMotorControl::enable(ArmSerialPort& serial, const ControlModes& modes)
{
    return sendAction(serial, modes, ActionEnable);
}

bool ArmMotorControl::disable(ArmSerialPort& serial, const ControlModes& modes)
{
    return sendAction(serial, modes, ActionDisable);
}

bool ArmMotorControl::setZero(ArmSerialPort& serial, const ControlModes& modes)
{
    return sendAction(serial, modes, ActionSetZero);
}

bool ArmMotorControl::sendAction(ArmSerialPort& serial,
                                 const ControlModes& modes,
                                 std::uint8_t action)
{
    ArmSerialPort::Payload payload{};

    std::uint8_t special_byte = EnableByte;
    if (action == ActionDisable)
        special_byte = DisableByte;
    else if (action == ActionSetZero)
        special_byte = SetZeroByte;

    const auto special_command = makeSpecialCommand(special_byte);

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        std::copy(special_command.begin(),
                  special_command.end(),
                  payload.begin() + static_cast<std::ptrdiff_t>(i * 8));
    }

    addProtocolMetadata(payload, modes, action);
    return serial.sendPayload(payload);
}

bool ArmMotorControl::write(ArmSerialPort& serial, const ArmLowCmd& low_cmd)
{
    ArmSerialPort::Payload payload{};

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
    {
        const auto& command = low_cmd.motor_cmd()[i];
        if (!validateMotorType(i, command.motor_type()))
            return false;

        if (!validateProtocolLimits(command, i))
            return false;

        std::array<std::uint8_t, 8> frame{};
        if (command.mode() == ArmControlMode::MIT)
            frame = packMitCommand(command);
        else if (command.mode() == ArmControlMode::PositionVelocity)
            frame = packPositionVelocityCommand(command);
        else
        {
            setLastError("Unsupported control mode for motor "
                         + std::to_string(i + 1U) + ".");
            return false;
        }

        std::copy(frame.begin(), frame.end(), payload.begin() + static_cast<std::ptrdiff_t>(i * 8));

        last_command_motor_types_[i] = command.motor_type();
    }

    ControlModes modes{};
    for (std::size_t i = 0; i < kArmMotorCount; ++i)
        modes[i] = low_cmd.motor_cmd()[i].mode();
    addProtocolMetadata(payload, modes, ActionCommand);
    return serial.sendPayload(payload);
}

bool ArmMotorControl::read(ArmSerialPort& serial, ArmLowState& low_state)
{
    ArmSerialPort::Payload payload{};

    if (!serial.getReceivedPayload(payload))
        return false;

    protocol_v2_detected_ = payload[ProtocolMarkerIndex] == ProtocolMarker;
    if (protocol_v2_detected_)
    {
        stm32_watchdog_tripped_ = (payload[ProtocolModeMaskIndex] & 0x80U) != 0U;
        last_acknowledged_control_ = payload[ProtocolControlIndex];
    }

    for (std::size_t i = 0; i < kArmMotorCount; ++i)
        if (!decodeMotorFeedback(payload,
                                 i,
                                 last_command_motor_types_[i],
                                 low_state.motor_state_[i]))
            low_state.motor_state_[i].valid_ = false;

    return true;
}

bool ArmMotorControl::protocolV2Detected() const noexcept
{
    return protocol_v2_detected_;
}

bool ArmMotorControl::stm32WatchdogTripped() const noexcept
{
    return stm32_watchdog_tripped_;
}

std::uint8_t ArmMotorControl::lastAcknowledgedControl() const noexcept
{
    return last_acknowledged_control_;
}

std::array<std::uint8_t, 8> ArmMotorControl::packMitCommand(const ArmMotorCmd& command)
{
    const auto& protocol = protocolLimits(command.motor_type());

    const float q = command.q();
    const float dq = command.dq();
    const float kp = command.kp();
    const float kd = command.kd();
    const float tau = command.tau();

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

std::array<std::uint8_t, 8> ArmMotorControl::packPositionVelocityCommand(
  const ArmMotorCmd& command) const
{
    std::array<std::uint8_t, 8> frame{};
    const float position = command.q();
    const float velocity_limit = command.dq();
    static_assert(sizeof(float) == 4, "Position-velocity protocol requires float32");
    std::memcpy(frame.data(), &position, sizeof(position));
    std::memcpy(frame.data() + sizeof(position), &velocity_limit, sizeof(velocity_limit));
    return frame;
}

void ArmMotorControl::addProtocolMetadata(ArmSerialPort::Payload& payload,
                                          const ControlModes& modes,
                                          std::uint8_t action)
{
    payload[ProtocolMarkerIndex] = ProtocolMarker;
    payload[ProtocolModeMaskIndex] = modeMask(modes);
    payload[ProtocolControlIndex] = static_cast<std::uint8_t>(
      ((sequence_++ & 0x3FU) << 2U) | (action & 0x03U));
}

std::uint8_t ArmMotorControl::modeMask(const ControlModes& modes)
{
    std::uint8_t mask = 0;
    for (std::size_t i = 0; i < kArmMotorCount; ++i)
        if (modes[i] == ArmControlMode::PositionVelocity)
            mask |= static_cast<std::uint8_t>(1U << i);
    return mask;
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

    const bool protocol_v2 = payload[ProtocolMarkerIndex] == ProtocolMarker;
    const bool stm_feedback_valid = !protocol_v2
                                    || (payload[ProtocolModeMaskIndex]
                                        & static_cast<std::uint8_t>(1U << motor_index)) != 0U;
    state.valid_ = stm_feedback_valid && state.motor_id_ == expected_id;

    return state.valid_;
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

bool ArmMotorControl::validateProtocolLimits(const ArmMotorCmd& command,
                                             std::size_t motor_index)
{
    if (motor_index >= kArmMotorCount)
    {
        setLastError("Invalid motor index.");
        return false;
    }

    const auto& protocol = protocolLimits(command.motor_type());
    const std::string prefix = "Motor " + std::to_string(motor_index + 1U) + ": ";

    const auto in_range = [](float value, float minimum, float maximum) {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    };

    if (!in_range(command.q(), protocol.position_min, protocol.position_max))
    {
        setLastError(prefix + "position is outside protocol limits.");
        return false;
    }
    const bool velocity_valid = command.mode() == ArmControlMode::PositionVelocity
                                  ? std::isfinite(command.dq()) && command.dq() > 0.0F
                                      && command.dq() <= protocol.velocity_max
                                  : in_range(command.dq(), protocol.velocity_min,
                                             protocol.velocity_max);
    if (!velocity_valid)
    {
        setLastError(prefix + "velocity is outside protocol limits.");
        return false;
    }
    if (command.mode() == ArmControlMode::MIT
        && !in_range(command.tau(), protocol.torque_min, protocol.torque_max))
    {
        setLastError(prefix + "torque is outside protocol limits.");
        return false;
    }
    if (command.mode() == ArmControlMode::MIT
        && !in_range(command.kp(), protocol.kp_min, protocol.kp_max))
    {
        setLastError(prefix + "kp is outside protocol limits.");
        return false;
    }
    if (command.mode() == ArmControlMode::MIT
        && !in_range(command.kd(), protocol.kd_min, protocol.kd_max))
    {
        setLastError(prefix + "kd is outside protocol limits.");
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
