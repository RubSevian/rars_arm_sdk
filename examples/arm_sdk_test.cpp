#include "include/arm_motor_control.hpp"
#include "include/arm_serial_port.hpp"
#include "include/arm_types.hpp"

#include <QApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <array>
#include <cmath>
#include <cstddef>

namespace
{

constexpr int kSliderMinimum = -10000;
constexpr int kSliderMaximum = 10000;

constexpr int kSendPeriodMs		= 5;  // 200 Гц
constexpr int kFeedbackPeriodMs = 30; // около 33 Гц

constexpr float kPi = 3.14159265358979323846F;

constexpr std::array<float, rars_arm::kArmMotorCount> kMotorKp{
  60.0F, // M1 DM4340
  60.0F, // M2 DM4340
  60.0F, // M3 DM4340
  20.0F, // M4 DM4310
  20.0F, // M5 DM4310
  20.0F, // M6 DM4310
  20.0F	 // M7 DM4310
};

constexpr std::array<float, rars_arm::kArmMotorCount> kMotorKd{
  1.0F, // M1
  1.0F, // M2
  1.0F, // M3
  1.0F, // M4
  1.0F, // M5
  1.0F, // M6
  1.0F	// M7
};

float sliderToRadians(int value)
{
    return static_cast<float>(value) * (2.0F * kPi) / static_cast<float>(kSliderMaximum);
}

void addMotorRow(QGridLayout*	grid,
                 int			row,
                 const QString& name,
                 QSlider*&		slider,
                 QLabel*&		target_label,
                 QLabel*&		current_label,
                 QLabel*&		error_label,
                 QLabel*&		torque_label,
                 QLabel*&		mos_temperature_label,
                 QLabel*&		rotor_temperature_label)
{
    auto* name_label = new QLabel(name);

    target_label = new QLabel(QStringLiteral("target: 0.000 rad"));

    current_label = new QLabel(QStringLiteral("current: 0.000 rad"));

    error_label = new QLabel(QStringLiteral("err: 0.000 rad"));

    torque_label = new QLabel(QStringLiteral("tau: 0.000 Nm"));

    mos_temperature_label = new QLabel(QStringLiteral("MOS: -- C"));

    rotor_temperature_label = new QLabel(QStringLiteral("Rotor: -- C"));

    slider = new QSlider(Qt::Horizontal);
    slider->setRange(kSliderMinimum, kSliderMaximum);
    slider->setValue(0);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setTickInterval(2000);

    grid->addWidget(name_label, row, 0);
    grid->addWidget(slider, row, 1);
    grid->addWidget(target_label, row, 2);
    grid->addWidget(current_label, row, 3);
    grid->addWidget(error_label, row, 4);
    grid->addWidget(torque_label, row, 5);
    grid->addWidget(mos_temperature_label, row, 6);
    grid->addWidget(rotor_temperature_label, row, 7);
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rars_arm::ArmSerialPort serial("/dev/ttyACM0", 921600);

    rars_arm::ArmMotorControl motor_control;
    rars_arm::ArmLowCmd		  arm_command;
    rars_arm::ArmLowState	  arm_state;

    if (!serial.open())
    {
        QMessageBox::critical(nullptr,
                              QStringLiteral("Serial error"),
                              QString::fromStdString(serial.lastError()));

        return 1;
    }

    if (!serial.startReceiving())
    {
        QMessageBox::critical(nullptr,
                              QStringLiteral("Receiving error"),
                              QString::fromStdString(serial.lastError()));

        serial.close();
        return 1;
    }

    for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
    {
        auto& command = arm_command.motor_cmd()[i];

        command.motor_type() =
          i < 3 ? rars_arm::ArmMotorType::DM4340 : rars_arm::ArmMotorType::DM4310;

        command.mode() = rars_arm::ArmControlMode::MIT;

        command.q()	  = 0.0F;
        command.dq()  = 0.0F;
        command.kp()  = kMotorKp[i];
        command.kd()  = kMotorKd[i];
        command.tau() = 0.0F;
    }

    QWidget window;
    window.setWindowTitle(QStringLiteral("Damiao 7 Motor Control"));

    auto* main_layout = new QVBoxLayout(&window);

    auto* grid = new QGridLayout();

    std::array<QSlider*, rars_arm::kArmMotorCount> sliders{};

    std::array<QLabel*, rars_arm::kArmMotorCount> target_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> current_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> error_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> torque_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> mos_temperature_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> rotor_temperature_labels{};

    for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
    {
        const QString motor_type = i < 3 ? QStringLiteral("DM4340") : QStringLiteral("DM4310");

        addMotorRow(grid,
                    static_cast<int>(i),
                    QStringLiteral("M%1 %2").arg(i + 1).arg(motor_type),
                    sliders[i],
                    target_labels[i],
                    current_labels[i],
                    error_labels[i],
                    torque_labels[i],
                    mos_temperature_labels[i],
                    rotor_temperature_labels[i]);

        QObject::connect(sliders[i],
                         &QSlider::valueChanged,
                         [&arm_command, &target_labels, i](int value) {
                             const float position = sliderToRadians(value);

                             arm_command.motor_cmd()[i].q() = position;

                             target_labels[i]->setText(
                               QStringLiteral("target: %1 rad").arg(position, 0, 'f', 3));
                         });
    }

    auto* button_layout = new QHBoxLayout();

    auto* enable_button = new QPushButton(QStringLiteral("Enable"));

    auto* disable_button = new QPushButton(QStringLiteral("Disable"));

    auto* zero_button = new QPushButton(QStringLiteral("Set Zero"));

    auto* center_button = new QPushButton(QStringLiteral("Center sliders"));

    button_layout->addWidget(enable_button);
    button_layout->addWidget(disable_button);
    button_layout->addWidget(zero_button);
    button_layout->addWidget(center_button);

    main_layout->addLayout(grid);
    main_layout->addLayout(button_layout);

    QTimer send_timer;
    send_timer.setInterval(kSendPeriodMs);
    send_timer.setTimerType(Qt::PreciseTimer);

    QTimer feedback_timer;
    feedback_timer.setInterval(kFeedbackPeriodMs);

    bool motors_enabled = false;

    QObject::connect(&feedback_timer, &QTimer::timeout, [&]() {
        if (!motor_control.read(serial, arm_state))
            return;

        for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
        {
            const auto& state = arm_state.motor_state()[i];

            const float target = arm_command.motor_cmd()[i].q();

            const float current = state.q();

            const float position_error = target - current;

            current_labels[i]->setText(QStringLiteral("current: %1 rad").arg(current, 0, 'f', 3));

            error_labels[i]->setText(QStringLiteral("err: %1 rad").arg(position_error, 0, 'f', 3));

            torque_labels[i]->setText(QStringLiteral("tau: %1 Nm").arg(state.tau(), 0, 'f', 3));

            mos_temperature_labels[i]->setText(
              QStringLiteral("MOS: %1 C").arg(state.mos_temperature(), 0, 'f', 0));

            rotor_temperature_labels[i]->setText(
              QStringLiteral("Rotor: %1 C").arg(state.rotor_temperature(), 0, 'f', 0));
        }
    });

    feedback_timer.start();

    QObject::connect(
      &send_timer,
      &QTimer::timeout,
      [&]() {
          if (!motors_enabled)
              return;

          if (!motor_control.write(serial, arm_command))
          {
              send_timer.stop();
              motors_enabled = false;

              motor_control.disable(serial);

              QMessageBox::critical(&window,
                                    QStringLiteral("Write error"),
                                    QString::fromStdString(motor_control.lastError()));
          }
      });

    QObject::connect(
      enable_button,
      &QPushButton::clicked,
      [&]() {
          if (motors_enabled)
          {
              return;
          }

          for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
          {
              arm_command.motor_cmd()[i].q() = sliderToRadians(sliders[i]->value());
          }

          if (!motor_control.enable(serial))
          {
              QMessageBox::critical(&window,
                                    QStringLiteral("Enable error"),
                                    QString::fromStdString(motor_control.lastError()));

              return;
          }

          QTimer::singleShot(
            200,
            [&]() {
                if (!motor_control.enable(serial))
                {
                    QMessageBox::critical(&window,
                                          QStringLiteral("Enable error"),
                                          QString::fromStdString(motor_control.lastError()));

                    return;
                }

                motors_enabled = true;
                send_timer.start();
            });
      });

    QObject::connect(
      disable_button,
      &QPushButton::clicked,
      [&]() {
          send_timer.stop();
          motors_enabled = false;

          for (int attempt = 0; attempt < 5; ++attempt)
          {
              QTimer::singleShot(
                attempt * 50,
                [&]() { motor_control.disable(serial); });
          }
      });

    QObject::connect(
      zero_button,
      &QPushButton::clicked,
      [&]() {
          const auto answer = QMessageBox::warning(&window,
                                                   QStringLiteral("Set Zero"),
                                                   QStringLiteral("Текущее положение всех моторов "
                                                                  "будет записано как нулевое.\n"
                                                                  "Продолжить?"),
                                                   QMessageBox::Yes | QMessageBox::No,
                                                   QMessageBox::No);

          if (answer != QMessageBox::Yes)
          {
              return;
          }

          if (!motor_control.setZero(serial))
          {
              QMessageBox::critical(&window,
                                    QStringLiteral("Set Zero error"),
                                    QString::fromStdString(motor_control.lastError()));
          }
      });

    QObject::connect(center_button, &QPushButton::clicked, [&]() {
        for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
        {
            sliders[i]->setValue(0);

            arm_command.motor_cmd()[i].q() = 0.0F;
        }
    });

    window.resize(1100, 420);
    window.show();

    const int result = app.exec();

    send_timer.stop();
    feedback_timer.stop();

    for (int attempt = 0; attempt < 3; ++attempt)
        motor_control.disable(serial);

    serial.stopReceiving();
    serial.close();

    return result;
}
