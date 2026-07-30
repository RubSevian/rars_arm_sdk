cmake_minimum_required(VERSION 3.16)

project(rars_arm_sdk VERSION 0.1.0 LANGUAGES CXX)

option(RARS_ARM_BUILD_QT_EXAMPLE "Build the Qt hardware test application" ON)
option(RARS_ARM_BUILD_PYTHON "Build the pybind11 Python module" OFF)
option(RARS_ARM_BUILD_TESTING "Build SDK unit tests" OFF)

include(GNUInstallDirs)

find_package(Threads REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
find_library(LIBSERIAL_LIBRARY NAMES serial)
if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
    message(FATAL_ERROR
        "LibSerial development files were not found. Install the package that "
        "provides libserial/SerialPort.h and libserial.so (on Ubuntu: "
        "libserial-dev)."
    )
endif()

add_library(rars_arm_sdk
    src/arm_motor_control.cpp
    src/arm_serial_port.cpp
    src/rars_arm.cpp
)
add_library(rars_arm::sdk ALIAS rars_arm_sdk)

target_compile_features(rars_arm_sdk PUBLIC cxx_std_17)
target_include_directories(rars_arm_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PUBLIC
        ${LIBSERIAL_INCLUDE_DIR}
)
target_link_libraries(rars_arm_sdk
    PUBLIC
        Threads::Threads
        ${LIBSERIAL_LIBRARY}
)
set_target_properties(rars_arm_sdk PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME sdk
    POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rars_arm_sdk PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(RARS_ARM_BUILD_QT_EXAMPLE)
    find_package(Qt5 REQUIRED COMPONENTS Widgets)

    add_executable(rars_arm_qt_test examples/arm_sdk_test.cpp)
    target_link_libraries(rars_arm_qt_test
        PRIVATE
            rars_arm::sdk
            Qt5::Widgets
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(rars_arm_qt_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

if(RARS_ARM_BUILD_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(rars_arm_py python/bindings.cpp)
    target_link_libraries(rars_arm_py PRIVATE rars_arm::sdk)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(rars_arm_py PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    install(
        TARGETS rars_arm_py
        LIBRARY DESTINATION
            ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
    )
endif()

if(RARS_ARM_BUILD_TESTING)
    include(CTest)
    enable_testing()
endif()

install(
    TARGETS rars_arm_sdk
    EXPORT rars_arm_sdkTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)
install(
    EXPORT rars_arm_sdkTargets
    FILE rars_arm_sdkTargets.cmake
    NAMESPACE rars_arm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rars_arm_sdk
)
export(
    EXPORT rars_arm_sdkTargets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/rars_arm_sdkTargets.cmake
    NAMESPACE rars_arm::
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/rars_arm_sdkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/rars_arm_sdkConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rars_arm_sdk
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/rars_arm_sdkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/rars_arm_sdkConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/rars_arm_sdkConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rars_arm_sdk
)
