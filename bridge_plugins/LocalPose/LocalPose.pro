QT -= gui
TARGET = LocalPose
CONFIG += plugin release c++11
TEMPLATE = lib

INCLUDEPATH += include

SOURCES += src/local_pose_plugin.cpp

HEADERS += include/local_pose_plugin.h