# rars_arm_sdk

Полная схема уровней и назначение каждого файла приведены в
[`ARCHITECTURE.md`](ARCHITECTURE.md).

Независимая от ROS и Qt библиотека C++17 для управления семью моторами
манипулятора RARS01. Моторы `0..5` соответствуют суставам руки, мотор `6` —
грипперу.

## Архитектура

- `include/rars_arm.hpp` — основной API `RarsArm` и конфигурация;
- `include/arm_motor_control.hpp` — упаковка протокола моторов;
- `include/arm_serial_port.hpp` — последовательный транспорт;
- `src/` — реализация библиотеки;
- `examples/arm_sdk_test.cpp` — отдельная Qt-проверка оборудования;
- `python/bindings.cpp` — Python API через pybind11.

SDK ничего не знает про ROS, MoveIt, URDF и RViz.

## Зависимости

```bash
sudo apt install cmake g++ libserial-dev
```

Для Qt-примера дополнительно:

```bash
sudo apt install qtbase5-dev
```

Для Python-модуля дополнительно нужны Python development files и pybind11.

## Сборка C++ библиотеки

```bash
cmake -S . -B build -DRARS_ARM_BUILD_QT_EXAMPLE=OFF
cmake --build build
```

Результат: `build/librars_arm_sdk.a`.

## Qt-проверка

```bash
cmake -S . -B build -DRARS_ARM_BUILD_QT_EXAMPLE=ON
cmake --build build
./build/rars_arm_qt_test
```

Qt-пример работает с реальными моторами. Перед `enable` робот должен находиться
в безопасной зоне. В каждой строке выбирается режим конкретного мотора:
`POS_VEL` или `MIT`. Переключатели блокируются после `Enable` и снова доступны
после успешного `Disable`. Период отправки вычисляется из
`ArmConfiguration.command_rate_hz` (по умолчанию `100 Hz`).

## Подключение из другого CMake-проекта

```cmake
find_package(rars_arm_sdk CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE rars_arm::sdk)
```

При конфигурации проекта укажите build-каталог SDK:

```bash
cmake -S . -B build \
  -Drars_arm_sdk_DIR=/home/ruben/Rars/rars_arm_sdk/build
```

Установка SDK не требуется. Для проверки установочной структуры можно
использовать staging внутри `build/`:

```bash
cmake --install build --prefix build/stage
```

## Конфигурация

`ArmConfiguration` содержит:

| Поле | Назначение |
|---|---|
| `port_name`, `baud_rate` | последовательный порт |
| `default_kp[7]`, `default_kd[7]` | коэффициенты MIT position-команды |
| `control_modes[7]` | режим каждого мотора: `POSITION_VELOCITY` или `MIT` |
| `position_velocity_limits[7]` | предел скорости для POS_VEL, рад/с |
| `command_rate_hz` | частота, с которой вызывающий код должен отправлять команды |
| `motors[i].type` | `DM4340` или `DM4310` |
| `motors[i].direction` | преобразование знака, только `+1` или `-1` |
| `motors[i].zero_offset` | смещение между joint и motor координатами |
| `joint_position_min/max` | аппаратный диапазон joint position |
| `joint_velocity_max` | аппаратный предел скорости |
| `joint_torque_max` | аппаратный предел момента |
| `feedback_watchdog_enabled` | включение watchdog |
| `feedback_timeout` | максимально допустимый возраст feedback |
| `initial_feedback_grace` | ожидание первого feedback после команды |

SDK проверяет `kp` в диапазоне `[0, 500]`, `kd` в `[0, 5]`, допустимость
таймаутов, motor type, direction, offset и соответствие limit-параметров
протоколу выбранного мотора.

Штатно суставы `1..6` работают в `POSITION_VELOCITY`, а мотор гриппера `7` —
в `MIT`. Режим можно изменить отдельно для каждого мотора через
`setControlModes()`/`set_control_modes()`, но только до `enable` или после
`disable`.

`command_rate_hz` не создаёт скрытый поток внутри библиотеки: частоту соблюдает
внешний control loop. Для рабочего grasp-драйвера используется `100 Hz`.
STM32 выполняет независимый watchdog `250 ms`, поэтому зависание процесса на
Linux не отключает защиту платы.

## C++ API

```cpp
#include <rars_arm.hpp>

rars_arm::ArmConfiguration config;
config.port_name = "/dev/ttyACM0";
config.baud_rate = 921600;
config.default_kp[0] = 60.0F;
config.default_kd[0] = 1.0F;
config.command_rate_hz = 100.0F;
config.control_modes[0] = rars_arm::ArmControlMode::PositionVelocity;
config.position_velocity_limits[0] = 0.5F;
config.motors[0].direction = 1.0F;
config.motors[0].zero_offset = 0.0F;

rars_arm::RarsArm arm(config);
if (!arm.connect() || !arm.enable())
    throw std::runtime_error(arm.lastError());

rars_arm::RarsArm::MotorValues target{};
if (!arm.sendPositionTargets(target))
    throw std::runtime_error(arm.lastError());

rars_arm::JointState state;
if (arm.tryReadJointState(state)) {
    // state.position, state.velocity, state.torque, state.valid
}

arm.disable();
arm.disconnect();
```

`setZero()` разрешён только при выключенных моторах.

## Python API

```bash
cmake -S . -B build-python \
  -DRARS_ARM_BUILD_QT_EXAMPLE=OFF \
  -DRARS_ARM_BUILD_PYTHON=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build-python
```

Пример:

```python
import rars_arm_py as sdk

config = sdk.ArmConfiguration()
config.port_name = "/dev/ttyACM0"
config.default_kp = [60.0, 60.0, 60.0, 20.0, 20.0, 20.0, 20.0]
config.default_kd = [1.0] * sdk.MOTOR_COUNT
config.motor(0).direction = 1.0
config.motor(0).zero_offset = 0.0

arm = sdk.RarsArm(config)
if not arm.connect() or not arm.enable():
    raise RuntimeError(arm.last_error)

arm.send_position_targets([0.0] * sdk.MOTOR_COUNT)
state = arm.try_read_joint_state()

arm.disable()
arm.disconnect()
```

Для смешанной команды (POS_VEL руки плюс MIT гриппера) используется
`send_configured(...)`. `send_mit(...)` теперь предназначен только для случая,
когда все семь моторов были заранее переведены в MIT.

## Протокол платы

SDK использует USB-протокол v2, не изменяя размер старого кадра 64 байта. В
последних трёх байтах payload передаются маркер версии, побитовая маска режимов,
команда enable/disable/set-zero и номер пакета. Полное описание находится в
`Light_lift_arm_6dof_h7_code/README_RARS_CONTROL.md` в общем workspace.

Запуск из build-каталога:

```bash
PYTHONPATH=build-python python3 my_script.py
```

## Границы безопасности

SDK проверяет команды и feedback, но не заменяет аппаратную защиту STM32,
аварийное отключение питания и механические ограничители. Watchdog SDK запрещает
новые команды после потери feedback; `ros2_control`-плагин дополнительно
отключает моторы при такой ошибке.
