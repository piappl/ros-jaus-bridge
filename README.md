# ros-jaus-bridge
## Description
Bi-directional communication bridge between ROS and JAUS. ROS/Jaus messages are translated to Jaus/ROS messages and then sent to specific ROS topic or Jaus address. 

Every message translation from one message type (e.g. ROS _geometry_msgs/Pose_) to other message type (e.g. _openjaus/mobility/SetLocalPose_) and vice versa is performed by specific translation plugin. User can write its own plugins to fulfil its needs regarding message translations.

Compiled translations plugins are loaded automatically and are used by bridge when they are needed.

**This repository contains of three packages:**
- ros_custom_msgs,
- bridge_plugins,
- ros-jaus-bridge.

See detailed information and usage instructions of each package below.
### ros_custom_msgs
It is ROS package, which can be used to build ROS custom messages. It is not obligatory to use this package. User can use its own package to create and generate ROS custom messages.

**Usage:**
1. Place your custom ROS msg file in _msg_ directory.
2. Open _CMakeLists.txt_ file and add your msg file name to _add_message_files(...)_ function, e.g.
   ```sh
   add_message_files(
      FILES
      MyRosMessage.msg
   )
    ```
`Do not forget to add *.msg extension!`
3. Build package with catkin.
4. Your custom message header file will be placed in ROS workspace _devel/include_ directory.
### bridge_plugins
This package (directory) contains all example translations plugins. 

**Creating custom bridge plugin:**

Bridge plugins are based on Qt plugins, so to create custom plugin Qt library should be installed in system.
1. Copy and paste _PluginTemplate_ directory. Rename it to your plugin name, e.g. _MyPlugin_.
2. Rename file _PluginTemplate.json_ to your plugin name, e.g. _MyPlugin.json_.
3. You can open this file and change plugin version.
   ```sh
   {
      "version": "1.0"
   }
   ```
4. Rename files _include/plugin_template.h_ and _src/plugin_template.cpp_ to your plugin name, e.g. _my_plugin.h_, _my_plugin_cpp_.
5. Rename _PluginTemplate.pro_ to _MyPlugin.pro_.
6. Edit file _MyPlugin.pro_ and set correct paths to files:
   ```sh
   SOURCES += src/my_plugin.cpp
   HEADERS += include/my_plugin.h
   ```
7. Open _include/my_plugin.h_ file. 

Each plugin should inherit from _QObject_ and _PluginInterface_ classes and each plugin should override virtual functions declared in _PluginInterface_ class:
- getType(),
- translateJausMsg(),
- translateRosMsg(),
- getMaintainer().

Plugin should also use _Q_PLUGIN_METADATA_ macro to specify plugin _*.json_ file.

`Important:`
User should also declare metatypes of ROS and JAUS message types, which will be used for translation, e.g.:
```sh
Q_DECLARE_METATYPE(openjaus::mobility::SetLocalPose)
Q_DECLARE_METATYPE(geometry_msgs::Pose)
```
Complete example header file should look like this:
```sh
#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <openjaus/mobility.h>
#include <geometry_msgs/Pose.h>

#include <tf/tf.h>

#include "../plugin_interface/plugin_interface.h"

Q_DECLARE_METATYPE(openjaus::mobility::SetLocalPose)
Q_DECLARE_METATYPE(geometry_msgs::Pose)

class MyPlugin : public QObject, public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "rj_bridge.PluginInterface" FILE "MyPlugin.json")
    Q_INTERFACES(PluginInterface)

public:
    PluginType getType() override;
    QVariant translateJausMsg(QVariant msg) const override;
    QVariant translateRosMsg(QVariant msg) const override;
    std::string getMaintainer() const override;
};

#endif // MY_PLUGIN_H
```
8. Open _src/my_plugin.cpp_.

In this file all functions should be defined. 

**getMaintainer() function**: this function should return _std::string_ with plugin's maintainer name, e.g. 
```sh
std::string MyPlugin::getMaintainer() const
{
    return std::string{"MyTheBestCompany"};
}
```
**getType() function**: this function should return plugin type. Plugin type can be retrieved by _translateType()_ template function defined in _PluginInterface_ class. First argument of template should be ROS message type, second argument should be Jaus message type, e.g.:
```sh
PluginType MyPlugin::getType()
{
    return translateType<geometry_msgs::Pose, openjaus::mobility::SetLocalPose>();
}
```
**translateRosMsg() and translateJausMsg() functions:** this two functions perform main action of plugin which is translating ROS or Jaus messages. Message is received in _QVariant_ object and should be casted to appropriate ROS or Jaus message type. After translation message is also returned in _QVariant_ object. Please see example plugins to get familiar with this mechanism. 

9. Build plugin in QtCreator. As build product you will receive library file _*.so_. All library files should be then copied to common directory, e.g.: _/home/user/bridge_plugins_.

### ros-jaus-bridge
This is ROS package, which loads plugins and creates ROS<->JAUS bridge(s).

**Usage:**
1. Open file _src/bridge/rj_bridge_node.cpp_. 
2. Main object of bridge is BridgeCore. Constructor of this objects takes four parameters. You can set bridge ROS node name in third parameter and plugins localization path (localization of _*.so_ files) in fourth parameter.
```sh
BridgeCore bridge_core(argc, argv, "ros_jaus_bridge", "/home/user/bridge_plugins/");
```
3. Then you can create bridges which you want. For example to create bridge from ROS to JAUS:
```sh
bridge_core.createBridgeR2J<geometry_msgs::Pose,
                openjaus::mobility::SetLocalPose>("local_pose", "jaus_cmp_local_pose");
```
This is template function: first template parameter should be ROS message type, and second parameter should be Jaus message type. 

This function takes two parameters. In first you can specify to which ROS topic bridge should subscribe and wait for ROS messages. Second parameters specifies Jaus destination address. When message is received on specified ROS topic it gets translated to Jaus message and then send to specified Jaus address.
4. To create Jaus->ROS bridge you should use _createBridgeJ2R()_ function, e.g.:
```sh
bridge_core.createBridgeJ2R<openjaus::mobility::SetWrenchEffort,
                rj_bridge_ros_msgs::SetWrenchEffort>("wrench_effort_ros", "wrench_effort_out");
```
In this template function first template parameter should be Jaus message type, second template parameter should be ROS message type. Same like _createBridgeR2J()_, this function also takes two parameters. In first you should specify name of bridge's Jaus component (other Jaus components will send messages to this address), in second you should set ROS output topic (where bridge should publish ROS messages).
5. It is also necessary to declare metatypes of used ROS and Jaus message types:
```sh
Q_DECLARE_METATYPE(openjaus::mobility::SetWrenchEffort)
Q_DECLARE_METATYPE(rj_bridge_ros_msgs::SetWrenchEffort)

Q_DECLARE_METATYPE(geometry_msgs::Pose)
Q_DECLARE_METATYPE(openjaus::mobility::SetLocalPose)
```
6. When everything is set you can run bridge. You can specify ROS node loop rate in parameter.
```sh
bridge_core.run(10);
```
7. Build this package using catkin and run it:
```sh
rosrun rj_bridge rj_bridge
```