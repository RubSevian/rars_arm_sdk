QT += widgets
CONFIG += c++17

SOURCES += src/arm_serial_port.cpp\
src/arm_motor_control.cpp\
examples/arm_sdk_test.cpp
           


HEADERS += include/arm_serial_port.hpp \
           include/arm_types.hpp\
           include/arm_motor_control.hpp

# Подключаем libserial (Linux)
LIBS += -lserial

# Если нужно явно указать include путь:
INCLUDEPATH += $$PWD


# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
