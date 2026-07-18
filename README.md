# rars_arm_sdk

C++17 library for communication with the RARS arm controller. The SDK core is
independent of Qt. The Qt application in `examples/` is an optional manual
hardware test.

On Ubuntu, install the build dependencies first:

```bash
sudo apt install cmake g++ libserial-dev qtbase5-dev
```

Qt is only required when `RARS_ARM_BUILD_QT_EXAMPLE` is enabled.

## Build the SDK and Qt test application

```bash
cmake -S . -B build -DRARS_ARM_BUILD_QT_EXAMPLE=ON
cmake --build build
./build/rars_arm_qt_test
```

The application expects `/dev/ttyACM0` at 921600 baud. Keep the robot in a safe
area and be ready to disable power before running hardware tests.

## Build only the SDK library

```bash
cmake -S . -B build -DRARS_ARM_BUILD_QT_EXAMPLE=OFF
cmake --build build
```

Other CMake projects can link the build-tree target as `rars_arm::sdk`. After
installation, use `find_package(rars_arm_sdk CONFIG REQUIRED)` and link the same
target.

The existing qmake project is retained for compatibility during the migration.

## High-level API

Applications should normally use `RarsArm` instead of managing the serial port
and protocol objects directly:

```cpp
#include <rars_arm/rars_arm.hpp>

rars_arm::RarsArm arm;
if (!arm.connect() || !arm.enable())
    throw std::runtime_error(arm.lastError());

rars_arm::RarsArm::MotorValues target{};
arm.sendPositionTargets(target);

rars_arm::ArmLowState state;
if (arm.tryReadState(state)) {
    // A fresh feedback packet is available.
}

const auto status = arm.communicationStatus();
// status.watchdog_tripped, feedback_age, valid_frames, invalid_frames, ...

arm.disable();
arm.disconnect();
```

`setZero()` is rejected while the motors are enabled. Command values containing
NaN or infinity are rejected before serialization.

The command watchdog is armed by the first successfully transmitted motion
command after `enable()`. This accounts for motors that produce no feedback
while disabled. It then allows one second for the first feedback packet and
requires feedback no older than 200 ms. It rejects new motion commands when it
trips; it does not automatically move or disable the arm. Both intervals and
the watchdog switch are available in `ArmConfiguration`.
