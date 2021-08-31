# ROS2 For Unity bridge by Robotec.ai

Custom ROS2 bridge plugin for SVL simulator version 2021.2.1+ with native C# implementation.

This plugin relies on binaries included in this repository [releases](https://github.com/RobotecAI/ROS2ForUnitySVLBridge/releases) page.

## Developer guide

To start developing `ROS2ForUnitySVLBridge` with Unity Editor:

1. Make sure you have LGSVL simulatior version 2021.2.1 (see https://www.svlsimulator.com/docs/installation-guide/build-instructions/ for more instructions about getting and launching SVL project).
2. Copy contents of this repository into `Assets/External/Bridges/ROS2ForUnitySVLBridge` folder of SVL simulator Unity project.
3. Unzip required binaries from [releases](https://github.com/RobotecAI/ROS2ForUnitySVLBridge/releases) to `Assets/Plugins` - you can use `deploy_unity_plugins.sh` script to do that for you:
```bash
cd Assets/External/Bridges/ROS2ForUnitySVLBridge
./deploy_unity_plugins.sh <ARCHIVE_PATH>
```

Use `ROS2ForUnitySVLBridgeInstance` and `ROS2ForUnitySVLBridgeFactory` classes to implement or change interface and `ROS2ForUnitySVLBridgeConversions` to add or modify existing conversions between SVL sensors and ROS2 messages.

### Building plugin

Building a simulator release with `ROS2ForUnitySVLBridge` plugin:

1. Source ROS2 foxy:
```bash
. /opr/ros/foxy/setup.bash
```
2. Set RMW middleware to `cyclonedds`:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
3. Update `LD_LIBRARY_PATH` to include plugins directory (this is due to Unity Editor limitation in linking certain libraries), please note OS subfolder `Linux/Windows`:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<SVL_PROJECT_PATH>/Assets/Plugins/<Linux/Windows>/x86_64
```
4. Launch Unity Editor directly:
```bash
<UNITY_EDITOR_PATH>/Unity -projectPath <SVL_PROJECT_PATH>
```
5. `Simulator -> Build` and check `ROS2ForUnitySVLBridge` under `Bridges`.
6. Click `Build`.

### Using `ROS2ForUnitySVLBridge` plugin:

SVL uses cloud-based web user interface for handling assets and simulations: https://wise.svlsimulator.com/. 

1. Find `ROS2ForUnitySVLBridge` under `https://wise.svlsimulator.com/plugins` and add it to your library.
2. install required binaries from github [releases](https://github.com/RobotecAI/ROS2ForUnitySVLBridge/releases) page into SVL simulator application:
   1. libraries from release archive `Plugins/x86_64/*` goes into `simulator_Data/Plugins` of svl simulator directory,
   2. libraries from release archive `Plugins/*.dll` goes into `simulator_Data/Managed` of svl simulator directory.

Then you can just use SVL web interface to set up bridge. See https://www.svlsimulator.com/docs for more informations on how to set up an SVL simulation.

**IMPORTANT** After making an app build from editor, you must manually copy soversion files (ending with library version `.so.X.Y.Z`) from editor `Plugins/<OS>/x86_64` to `simulator_Data/Plugins` folder. This is due to Unity for some reason doesn't copy them while deploying application.
