QT -= gui
TARGET = WrenchEffort
CONFIG += plugin release c++11
TEMPLATE = lib

INCLUDEPATH += include

SOURCES += src/wrench_effort_plugin.cpp

HEADERS += include/wrench_effort_plugin.h