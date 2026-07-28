#include "include/rars_arm.hpp"

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
#include <cstddef>

namespace
{

constexpr int kSliderMinimum = -10000;
constexpr int kSliderMaximum = 10000;

constexpr int kSendPeriodMs		= 5;  // 200 Гц
constexpr int kFeedbackPeriodMs = 30; // около 33 Гц

float sliderToPosition(int value, const rars_arm::MotorConfiguration& motor)
{
    if (value >= 0)
        return static_cast<float>(value) * motor.joint_position_max
               / static_cast<float>(kSliderMaximum);
    return static_cast<float>(-value) * motor.joint_position_min
           / static_cast<float>(kSliderMaximum);
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

    rars_arm::RarsArm arm;
    rars_arm::RarsArm::MotorValues target_positions{};
    rars_arm::JointState arm_state;

    if (!arm.connect())
    {
        QMessageBox::critical(nullptr,
                              QStringLiteral("Connection error"),
                              QString::fromStdString(arm.lastError()));

        return 1;
    }

    QWidget window;
    window.setWindowTitle(QStringLiteral("Damiao 7 Motor Control"));

    auto* main_layout = new QVBoxLayout(&window);

    auto* grid = new QGridLayout();

    auto* communication_label = new QLabel(
      QStringLiteral("feedback: waiting, valid: 0, invalid: 0, timeouts: 0"));

    std::array<QSlider*, rars_arm::kArmMotorCount> sliders{};

    std::array<QLabel*, rars_arm::kArmMotorCount> target_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> current_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> error_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> torque_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> mos_temperature_labels{};

    std::array<QLabel*, rars_arm::kArmMotorCount> rotor_temperature_labels{};

    for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
    {
        const QString motor_type = arm.configuration().motors[i].type
                                           == rars_arm::ArmMotorType::DM4340
                                     ? QStringLiteral("DM4340")
                                     : QStringLiteral("DM4310");
        const QString motor_name = i == rars_arm::kGripperMotorIndex
                                   ? QStringLiteral("Gripper M%1 %2").arg(i + 1).arg(motor_type)
                                   : QStringLiteral("Joint %1 M%1 %2").arg(i + 1).arg(motor_type);

        addMotorRow(grid,
                    static_cast<int>(i),
                    motor_name,
                    sliders[i],
                    target_labels[i],
                    current_labels[i],
                    error_labels[i],
                    torque_labels[i],
                    mos_temperature_labels[i],
                    rotor_temperature_labels[i]);

        QObject::connect(sliders[i],
                         &QSlider::valueChanged,
                         [&arm, &target_positions, &target_labels, i](int value) {
                             const float position = sliderToPosition(
                               value, arm.configuration().motors[i]);

                             target_positions[i] = position;

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
    main_layout->addWidget(communication_label);
    main_layout->addLayout(button_layout);

    QTimer send_timer;
    send_timer.setInterval(kSendPeriodMs);
    send_timer.setTimerType(Qt::PreciseTimer);

    QTimer feedback_timer;
    feedback_timer.setInterval(kFeedbackPeriodMs);

    bool motors_enabled = false;

    QObject::connect(&feedback_timer, &QTimer::timeout, [&]() {
        const auto communication = arm.communicationStatus();
        const QString feedback_age = communication.feedback_received
                                     ? QString::number(communication.feedback_age.count()) + " ms"
                                     : QStringLiteral("waiting");
        const QString watchdog_state = !communication.watchdog_armed
                                        ? QStringLiteral("idle")
                                        : communication.watchdog_tripped
                                            ? QStringLiteral("TRIPPED")
                                            : communication.feedback_received
                                                ? QStringLiteral("ok")
                                                : QStringLiteral("waiting");
        communication_label->setText(
          QStringLiteral("feedback: %1, watchdog: %2, valid: %3, invalid: %4, timeouts: %5, read errors: %6")
            .arg(feedback_age)
            .arg(watchdog_state)
            .arg(static_cast<qulonglong>(communication.valid_frames))
            .arg(static_cast<qulonglong>(communication.invalid_frames))
            .arg(static_cast<qulonglong>(communication.read_timeouts))
            .arg(static_cast<qulonglong>(communication.read_errors)));

        if (!arm.tryReadJointState(arm_state))
            return;

        for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
        {
            const float target = target_positions[i];

            const float current = arm_state.position[i];

            const float position_error = target - current;

            current_labels[i]->setText(QStringLiteral("current: %1 rad").arg(current, 0, 'f', 3));

            error_labels[i]->setText(QStringLiteral("err: %1 rad").arg(position_error, 0, 'f', 3));

            torque_labels[i]->setText(
              QStringLiteral("tau: %1 Nm").arg(arm_state.torque[i], 0, 'f', 3));

            mos_temperature_labels[i]->setText(
              QStringLiteral("MOS: %1 C").arg(arm_state.mos_temperature[i], 0, 'f', 0));

            rotor_temperature_labels[i]->setText(
              QStringLiteral("Rotor: %1 C").arg(arm_state.rotor_temperature[i], 0, 'f', 0));
        }
    });

    feedback_timer.start();

    QObject::connect(
      &send_timer,
      &QTimer::timeout,
      [&]() {
          if (!motors_enabled)
              return;

          if (!arm.sendPositionTargets(target_positions))
          {
              const std::string write_error = arm.lastError();
              send_timer.stop();
              motors_enabled = false;

              arm.disable();

              QMessageBox::critical(&window,
                                    QStringLiteral("Write error"),
                                    QString::fromStdString(write_error));
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
              target_positions[i] = sliderToPosition(
                sliders[i]->value(), arm.configuration().motors[i]);
          }

          if (!arm.enable())
          {
              QMessageBox::critical(&window,
                                    QStringLiteral("Enable error"),
                                    QString::fromStdString(arm.lastError()));

              return;
          }

          motors_enabled = true;
          send_timer.start();
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
                [&]() { arm.disable(); });
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

          if (!arm.setZero())
          {
              QMessageBox::critical(&window,
                                    QStringLiteral("Set Zero error"),
                                    QString::fromStdString(arm.lastError()));
          }
      });

    QObject::connect(center_button, &QPushButton::clicked, [&]() {
        for (std::size_t i = 0; i < rars_arm::kArmMotorCount; ++i)
        {
            sliders[i]->setValue(0);

            target_positions[i] = 0.0F;
        }
    });

    window.resize(1100, 420);
    window.show();

    const int result = app.exec();

    send_timer.stop();
    feedback_timer.stop();

    for (int attempt = 0; attempt < 3; ++attempt)
        arm.disable();

    arm.disconnect();

    return result;
}
