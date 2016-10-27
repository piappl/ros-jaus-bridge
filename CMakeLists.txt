cmake_minimum_required(VERSION 2.8.3)
project(rj_bridge)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")

find_package(catkin REQUIRED roscpp)
find_package(Qt5 COMPONENTS Core)

catkin_package()

# User must define these paths
set(QT_INCLUDE_PATH "/home/USER_NAME/Qt/5.6.0/5.6/gcc_64/include")
set(QT_LIB_PATH "/home/USER_NAME/Qt/5.6.0/5.6/gcc_64/lib")
set(JAUS_LIB_PATH "/home/USER_NAME/Libraries/org.openjaus.sdk.cpp/lib")

include_directories(include
                    include/bridge
                    include/plugin
                    ${catkin_INCLUDE_DIRS}
                    ${QT_INCLUDE_PATH})

link_directories(${QT_LIB_PATH}
                 ${JAUS_LIB_PATH})

add_executable(rj_bridge src/bridge/rj_bridge_node.cpp
                         src/bridge/bridge_core.cpp)
add_dependencies(rj_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS}
                           ${catkin_EXPORTED_TARGETS})
target_link_libraries(rj_bridge ${catkin_LIBRARIES}
                                openjaus openjaus-core
                                openjaus-mobility
                                Qt5Core)

add_executable(ros_publisher src/test_apps/ros_publisher.cpp)
target_link_libraries(ros_publisher ${catkin_LIBRARIES})

add_executable(ros_subscriber src/test_apps/ros_subscriber.cpp)
target_link_libraries(ros_subscriber ${catkin_LIBRARIES})

add_executable(jaus_publisher src/test_apps/jaus_publisher.cpp)
target_link_libraries(jaus_publisher openjaus openjaus-core openjaus-mobility)

add_executable(jaus_subscriber src/test_apps/jaus_subscriber.cpp)
target_link_libraries(jaus_subscriber openjaus openjaus-core openjaus-mobility)