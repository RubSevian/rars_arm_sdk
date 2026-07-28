#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace rars_arm
{

inline constexpr std::size_t kArmJointCount = 6;
inline constexpr std::size_t kGripperMotorIndex = 6;
inline constexpr std::size_t kArmMotorCount = kArmJointCount + 1;

enum class ArmMotorType : std::uint8_t
{
    Unknown = 0,
    DM4310,
    DM4340
};
enum class ArmControlMode : std::uint8_t
{
    MIT = 0x01
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

class ArmMotorCmd
{
public:

    ArmMotorType& motor_type() noexcept
    {
        return motor_type_;
    } // возврат неконстантной ссылки (можем изменять обьект мотора)

    ArmMotorType motor_type() const noexcept
    {
        return motor_type_;
    } // обратная связь

    ArmControlMode& mode() noexcept
    {
        return mode_;
    }

    ArmControlMode mode() const noexcept
    {
        return mode_;
    }

    float& q() noexcept
    {
        return q_;
    }

    float q() const noexcept
    {
        return q_;
    }

    float& dq() noexcept
    {
        return dq_;
    }

    float dq() const noexcept
    {
        return dq_;
    }

    float& kp() noexcept
    {
        return kp_;
    }

    float kp() const noexcept
    {
        return kp_;
    }

    float& kd() noexcept
    {
        return kd_;
    }

    float kd() const noexcept
    {
        return kd_;
    }

    float& tau() noexcept
    {
        return tau_;
    }

    float tau() const noexcept
    {
        return tau_;
    }

private:

    ArmMotorType   motor_type_ = ArmMotorType::Unknown;
    ArmControlMode mode_	   = ArmControlMode::MIT;

    float q_   = 0.0F;
    float dq_  = 0.0F;
    float kp_  = 0.0F;
    float kd_  = 0.0F;
    float tau_ = 0.0F;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

class ArmLowCmd
{ // команды 7 моторов
public:

    using CommandArray = std::array<ArmMotorCmd, kArmMotorCount>;

    CommandArray& motor_cmd() noexcept
    {
        return motor_cmd_;
    }

    const CommandArray& motor_cmd() const noexcept
    {
        return motor_cmd_;
    }

private:

    CommandArray motor_cmd_{};
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
class ArmMotorState
{
public:

    float q() const noexcept
    {
        return q_;
    }

    float dq() const noexcept
    {
        return dq_;
    }

    float tau() const noexcept
    {
        return tau_;
    }

    float mos_temperature() const noexcept
    {
        return mos_temperature_;
    }

    float rotor_temperature() const noexcept
    {
        return rotor_temperature_;
    }

    std::uint8_t error() const noexcept
    {
        return error_;
    }

    std::uint8_t motor_id() const noexcept
    {
        return motor_id_;
    }

    bool valid() const noexcept
    {
        return valid_;
    }

private:

    friend class ArmMotorControl;

    float q_   = 0.0F;
    float dq_  = 0.0F;
    float tau_ = 0.0F;

    float mos_temperature_	 = 0.0F;
    float rotor_temperature_ = 0.0F;

    std::uint8_t error_	   = 0;
    std::uint8_t motor_id_ = 0;
    bool		 valid_	   = false;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

class ArmLowState
{ // состояния семи моторов
public:

    using StateArray = std::array<ArmMotorState, kArmMotorCount>;

    const StateArray& motor_state() const noexcept
    {
        return motor_state_;
    }

private:

    friend class ArmMotorControl;

    StateArray motor_state_{};
};

} // namespace rars_arm
