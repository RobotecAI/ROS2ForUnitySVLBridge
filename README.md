# Ros2Native

Custom ROS2 bridge plugin for SVL simulator version 2021.2.1 with native C# implementation.

Please note that this bridge plugin is to be used with Autoware.auto.

This plugin relies on binaries `plugins.zip` included in this repository.

## Developer guide

To start developing `Ros2Native` with Unity Editor:

1. Make sure you have LGSVL simulatior version 2021.2.1 (see https://www.svlsimulator.com/docs/installation-guide/build-instructions/ for more instructions about getting and launching SVL project).
2. Copy contents of this repository into `Assets/External/Bridges/Ros2NativeBridge` folder of SVL simulator Unity project.
3. Unzip required binaries from `plugins.zip` to `Assets/Plugins` - you can use `deploy_unity_plugins.sh` script to do that for you:
```bash
cd Assets/External/Bridges/Ros2NativeBridge
./deploy_unity_plugins.sh
```

Use `Ros2NativeInstance` and `Ros2NativeFactory` classes to implement or change interface and `Ros2NativeConversions` to add or modify existing conversions between SVL sensors and ROS2 messages.

### Building plugin

Building a simulator release with `Ros2Native` plugin:

1. Source ROS2 foxy:
```bash
. /opr/ros/foxy/setup.bash
```
2. Set RMW middleware to `cyclonedds`:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
3. Update `LD_LIBRARY_PATH` to include plugins directory (this is due to Unity Editor limitation in linking certain libraries):
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<SVL_PROJECT_PATH>/Assets/Plugins/x86_64
```
4. Launch Unity Editor directly:
```bash
<UNITY_EDITOR_PATH>/Unity -projectPath <SVL_PROJECT_PATH>
```
5. `Simulator -> Build` and check `Ros2NativeBridge` under `Bridges`.
6. Click `Build`.

### Using `Ros2Native` plugin:

SVL uses cloud-based web user interface for handling assets and simulations: https://wise.svlsimulator.com/. 

`Ros2Native` bridge plugin:
1. needs to be deployed into SVL asset store, and
2. you have to place bundled binaries from `plugins.zip` into SVL simulator release:
   1. libraries from `plugins.zip` `Plugins/x86_64/*` goes into `simulator_Data/Plugins` of simulator release,
   2. libraries from `plugins.zip` `Plugins/*.dll` goes into `simulator_Data/Managed` of simulator release.

Then you can just use SVL web interface to set up bridge. See https://www.svlsimulator.com/docs for more informations on how to set up an SVL simulation.
