QT -= gui
TARGET = RobotInformation
CONFIG += plugin release c++11
TEMPLATE = lib

INCLUDEPATH += include

SOURCES += src/robot_information_plugin.cpp \
           src/jaus_msg/robot_information.cpp

HEADERS += include/robot_information_plugin.h \
           include/jaus_msg/robot_information.h