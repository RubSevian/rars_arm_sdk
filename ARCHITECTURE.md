# Архитектура `rars_arm_sdk`

## Назначение

`rars_arm_sdk` — независимая от ROS C++17-библиотека для обмена с STM32 и
управления семью DAMIAO-моторами: шесть суставов руки и мотор гриппера.

Основной поток данных:

```text
Приложение / ROS-плагин
        │ joint coordinates, kp/kd, torque
        ▼
RarsArm
        │ проверка конфигурации, direction/zero_offset, watchdog
        ▼
ArmMotorControl
        │ MIT-кодирование семи моторов
        ▼
ArmSerialPort
        │ кадр 64 байта, CRC16, поток приёма
        ▼
STM32 → CAN → DAMIAO motors
```

## Публичные уровни

### Высокоуровневый API

- `include/rars_arm.hpp` — публичный фасад `RarsArm`, конфигурация семи
  моторов, lifecycle подключения и enable/disable, MIT-команды, позиционные
  команды, преобразование feedback в координаты суставов и watchdog связи.
- `src/rars_arm.cpp` — реализация фасада, проверок, `direction`,
  `zero_offset`, программных лимитов и начальной обработки feedback.

Это рекомендуемый API для ROS-плагина, C++-приложений и Python bindings.

### Протокол моторов

- `include/arm_motor_control.hpp` — интерфейс кодирования команд DAMIAO,
  специальных команд enable/disable/set-zero и декодирования feedback.
- `src/arm_motor_control.cpp` — диапазоны протокола DM4310/DM4340,
  12/16-битное MIT-кодирование и проверка protocol limits.
- `include/arm_types.hpp` — низкоуровневые типы команды и состояния,
  `ArmMotorType`, `ArmControlMode`, размеры массивов и индекс гриппера.

Этот уровень не должен содержать ROS, MoveIt или описание геометрии робота.

### Serial transport

- `include/arm_serial_port.hpp` — RAII-интерфейс serial port, формат кадра,
  статистика связи и управление потоком приёма.
- `src/arm_serial_port.cpp` — LibSerial, упаковка/проверка кадров, CRC16,
  синхронизация потоков и хранение последнего полного feedback.

Транспорт знает только формат обмена с STM32 и не интерпретирует кинематику.

## Интеграционные файлы

- `python/bindings.cpp` — модуль `rars_arm_py` на pybind11; собирается только
  с `RARS_ARM_BUILD_PYTHON=ON`.
- `examples/arm_sdk_test.cpp` — Qt-приложение для ручной проверки SDK на
  настоящем оборудовании; собирается с `RARS_ARM_BUILD_QT_EXAMPLE=ON`.
- `rars_arm.pro` — совместимый Qt/qmake-проект для локального запуска примера.

## Сборка и экспорт

- `CMakeLists.txt` — библиотека `rars_arm_sdk`, alias target
  `rars_arm::sdk`, опциональные Qt/Python targets, установка headers и CMake
  package export.
- `cmake/rars_arm_sdkConfig.cmake.in` — шаблон установленного
  `rars_arm_sdkConfig.cmake`, через который внешние CMake-проекты выполняют
  `find_package(rars_arm_sdk CONFIG REQUIRED)`.
- `README.md` — пользовательская инструкция по сборке и использованию.

Генерируемые `build*` и `install` не являются частью архитектуры исходников и
не должны попадать в Git.

## Границы ответственности

SDK отвечает за:

- надёжную связь со STM32;
- MIT-протокол и типы моторов;
- enable/disable и watchdog;
- преобразование между motor и joint coordinates;
- независимые аппаратные проверки команд.

SDK не отвечает за:

- URDF/SRDF и кинематику;
- MoveIt и планирование;
- ROS topics/actions/services;
- lifecycle `ros2_control`;
- параметры конкретного запуска робота.

ROS-адаптер находится в
`reBotArmController_ROS2/src/rars_arm_hardware`.
