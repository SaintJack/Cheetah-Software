# Getting Started
This page explains how to download and install the software, run the MIT Controller, and how to create a controller of your own.
本文档主要是解释如何下载并安装这个软件，运行MIT Controller,以及如何创建一个你自己的controller.
# Install dependencies
#安装依赖库

Packages:
```
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
```

Others:
其他项（安装完以上依赖库，还需按照如下地址下载对应的文件，手动进行安装）
- LCM 1.3.1 (it says Java 6, but you can use newer) (https://lcm-proj.github.io/)
- Qt 5.10.0 or newer (requires the gamepad library) (https://www.qt.io/download-qt-installer)
- Eigen (http://eigen.tuxfamily.org/)

注意： Ubuntu 18.10 or 19.04 你可以使用以下命令行安装Qt
NOTE: on Ubuntu 18.10 or 19.04, you may instead install Qt with
```
sudo apt install libqt5 libqt5gamepad5
```


# Download and build code
#下载并构建代码
```
git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
cd scripts # for now, you must actually go into this folder 现在你必须确实的进入到这个文件夹
./make_types.sh # you may see an error like `rm: cannot remove...` but this is okay  你可能会看到一个错误，比如`rm: cannot remove...` ,但其实是好的
cd ..
mkdir build
cd build
cmake .. # there are still some warnings here
make -j
```


# Test 测试
The code in the `common` folder has some tests.  From the build folder, these can be run with `common/test-common`.   There are two tests which commonly fail:
在common文件夹中有一些测试用例。从build文件夹中，你可以使用命令`common/test-common`来运行。由2给测试用例通常会失败：
- OSQP - the solver itself is nondeterministic, so just run the test again and it should pass
-OSQP - 这个是不确定的，所以你可以再执行一遍测试，就会通过。
- CASADI - the solver loads a library at runtime and sometimes has issues finding it.  It's fine if this fails as we aren't using this solver yet.
- CASADI -这个解算器在运行时加载库，有时在查找库时有问题。如果这个失败了也没关系，我们现在还没有使用这个解算器。

# Joystick
We use the Logitech F310 controller.  There's a switch in the back, which should be in the "X" position.  The controller needs to reconnected if you change the switch position.  Also, the LED on the front near the mode button should be off.
(https://www.amazon.com/Logitech-940-000110-Gamepad-F310/dp/B003VAHYQY)
我们使用 Logitech F310 controller。

# Simulation Example模拟样例
The simulator default settings can be configured with `config/simulator-defaults.yaml` and `config/default-terrain.yaml`.  The default settings should be good for most uses, and the `default-terrain` file has commented out examples of how to add a mesh, box, and stairs.  The friction of the floor can be set from within the terrain file.
模拟器默认设置的配置文件在 `config/simulator-defaults.yaml` 和`config/default-terrain.yaml`中，默认设置对于大部分使用都应该是好的，`default-terrain`文件中以注释的方式呈现了如何添加网格、盒子以及楼梯作为地形的例子，地面的摩擦系数可以在该文件中设置。
To launch the simulator, first plug in your joystick, then run `sim/sim` from within the `build` folder.    Select "Mini Cheetah" and "Simulator", then click "Start".  Somewhere in the output, you should see
要运行模拟器，首先插入你的操作杆（上文讲的 Logitech F310），然后在`build`文件夹中执行 `sim/sim`。后面的参数选择"Mini Cheetah" and "Simulator"，然后点开始。在输出界面，你会看到：

```
[GameController] Found 1 joystick
```

The left panel allows you to change simulator settings.  The most useful setting is `simulation_speed`.  Defaults are loaded `simulator-defaults.yaml` when the simulator opens.  The settings file you load must have exactly the same set of parameters as is defined in the code.  You must recompile `sim` if you add or remove a parameter. You can save and load the parameters at any time, but note that the `use_spring_damper` setting will not take effect unless you restart the simulator.
左侧面板可以允许你更改模拟器的设置。大部分有用的设置是 `simulation_speed`。在打开模拟器时默认加载 `simulator-defaults.yaml`文件。你所加载的设置文件必须精确的和代码中设定的参数名称一致。如果你一处或者添加了新的参数，你必须重新编译‘sim’。你也可以保存和和加载参数，但是`use_spring_damper`参数的设定不会起效，除非你重新启动模拟器。

The center panel allows you to change robot settings which are not specific to the controller.  Defaults are loaded `mini-cheetah-defatuls.yaml` when Mini Cheetah is selected and you click "Start".  If you stop and start the simulator, the file will be reloaded.  The settings file you load must have exactly the same set of parameters as is defined in the code.  You must recompile `sim` if you add or remove a parameter.  Currently most of these parameters do nothing and many will be removed. The most useful setting is `cheater_mode`, which sends the robot code the current position/orientation/velocity of the robot, and `controller_dt`, which changes how frequently the control code runs. (The `controller_dt` setting still needs to be tested).
中间的面板允许你改变没有在控制器中确定的机器人的设置。如果你选择的是Mini Cheetah并点击开始，默认加载的是 `mini-cheetah-defatuls.yaml` 。如果你停止和启动模拟器，这个文件就会重新加载。你所加载的设置文件必须精确的和代码中设定的参数名称一致。如果你一处或者添加了新的参数，你必须重新编译‘sim’。当前这些参数什么都不做，其中很多可能会被移除。最有用的设置是`cheater_mode`，它会发送机器人当前的位置/朝向/速度， `controller_dt`参数可会改变重启代码的运行频率。（`controller_dt` 的设置还需要测试）

The right panel allows you to change parameters which are specific to your controller, called "User Parameters".  If your control code does not use user parameters, it will be ignored.  If your code does use user parameters, then you must load user configuration parameters that match the parameters your code is expecting, or your controller will not start.
右侧的面板允许你变更你的控制器的指定参数，比如"User Parameters"。如果你的控制代码并不需要用户参数，可以忽略。如果你的代码要用到用户参数化，那么你必须加载用户配置参数，并且要和代码中的一致，否则控制器不会启动。


To start the robot control code, run `user/MIT_Controller/mit_ctrl m s`.  The `m` argument is for mini-cheetah, and the `s` indicates it should connect to the simulator. This uses shared memory to communicate with the simulator. The simulator should start running, and the robot should move to a ready position.  In the center column of the simulator window, set control mode to 10.  Once the robot has stopped moving, set control mode 1.  Then, set control mode to 4, and the robot will start trotting.
要启动机器人控制代码，需要执行`user/MIT_Controller/mit_ctrl m s`。其中`m`参数指定的是mini-cheetah，`s` 指定的是连接的是模拟器。采用共享内存的方式进行与模拟器的通信。模拟器应该开始运行，机器人应该会移动到READY位置。在模拟器窗口中间的栏目，设置控制模式为10。一旦机器人停止移动，设置控制模式为1.然后设置控制模式为4，然后机器人就会开始trot步态。
You can use the joysticks to drive the robot around.  You will see two robots - the gray one is the actual robot position from the simulation, and the red one is the estimate of the robot's position from our state estimator.  Turning on "cheater_mode" will make the estimated position equal to the actual position.  To adjust the simulation view, you can click and drag on the screen and scroll. Press and hold `t` to make the simulation run as fast as possible.  Press the spacebar to turn on free camera mode.  You can use the w,a,s,d,r,f keys to move the camera around, and click and drag to adjust the orientation.
你可以使用操作杆驱动机器人行走。你将看到两个机器人-

# LCM
We use LCM (https://lcm-proj.github.io/) to connect the control interface to the actual mini cheetah hardware, and also as a debugging tool when running the simulator.  The `make_types.sh` script runs an LCM tool to generate C++ header files for the LCM data types.  When the simulator is running, you can run `scripts/launch_lcm_spy.sh` to open the LCM spy utility, which shows detailed information from the simulator and controller.  You can click on data streams to plot them, which is nice for debugging.  There is also a tool called `lcm-logger` which can save LCM data to a file.


# Writing a Robot Controller
To add your own robot controller, you should add a folder under `Cheetah-Software/user`, and add the folder to the `CMakeLists.txt` in `user`.  The `JPos_Controller` is an example of a very simple controller.  The `JPosUserParameters.h` file has an example of declaring two user parameters which can be adjusted from the simulator interface, but using user parameters is optional.  The `JPos_Controller.hpp` and `JPos_Controller.cpp` files are the actual controller, which should extend `RobotController`.  Notice that in the `JPos_Controller.hpp` file, the `getUserControlParameters` method retuns a pointer to the user parameters.  If you do not use user parameters, your `getUserControlParameters` should return `nullptr`.  Finally, your `main` function must be like the example main function in `main.cpp`.

The `runController` method of your controller will be called automatically, at 1 kHz.  Here, you have access to the following:

- `_quadruped` : contains constant parameters about the robot (link lengths, gear ratios, inertias...).  The `getHipLocation` function returns the location of the "hip" in the body coordinate system.  The x-axis points forward, y-axis to the left, and z-axis up.  The legs are ordered like this

```
FRONT
1 0  RIGHT
3 2
BACK
```

- `_model` : a dynamics model of the robot.  This can be used to compute forward kinematics, Jacobians, etc...
- `_legController`: Interface to the robot's legs. This data is syncronized with the hardware at around 700 Hz. There are multiple ways to control the legs, and the result from all the controllers are added together.
    - `commands[leg_id].tauFeedForward` : Leg torque (Nm, at the joint).  Order is ab/ad, hip, knee.
    - `commands[leg_id].forceFeedForward` : Force to apply at foot (N), in hip frame. (Same orientation as body frame, origin is the hip)
    - `commands[leg_id].qDes` : Desired joint position for joint PD controller (radians). Order is ab/ad, hip, knee.  `(0,0,0)` is leg pointing straight down.
    - `commands[leg_id].qdDes` : Desired joint velocity for joint PD controller (rad/sec).
    - `commands[leg_id].pDes, vDes` : Desired foot position/velocity for cartesian PD controller (meters, hip frame)
    - `commands[leg_id].kpCartesian, kdCartesian, kpJoint, kdJoint` : Gains for PD controllers (3x3 matrix).  Use the diagonal entries only.
    - `datas[leg_id].q` : Leg joint encoder (radians).  Order is ab/ad, hip, knee.  `(0,0,0)` is leg pointing straight down.
    - `datas[leg_id].qd` : Leg joint velocity (radians/sec).  Same order as `q`.
    - `datas[leg_id].p`  : Foot cartesian position, in hip frame. (Same orientation as body frame, origin is the hip)
    - `datas[leg_id].v`  : Foot cartesian velocity, in hip frame. 
    - `datas[leg_id].tau` : Estimate of motor torque from combination of all controllers
The joint PD control actually runs at 40 kHz on the motor controllers.
- `_stateEstimate, _stateEstimatorContainer` The result and interface for the provided state estimator.  If you provide the contact state of the robot (which feet are touching the ground), it will determine the robot's position/velocity in the world.
- `_driverCommand` : inputs from the game pad.
- `_controlParameters` : values from the center robot control parameters panel
- `_visualizationData` : interface to add debugging visualizations to the simulator window
- `_robotType` : If you are the mini Cheetah or Cheetah 3 robot.


If you would like to see more of how this works, look at the `robot` folder.  The `RobotRunner` class actually runs the control code, and connects it with either the `HardwareBridge` or `SimulationBridge`.  The code in the `rt` folder actually interacts with the hardware.


# Unfinished
- Verify that the controller update rate can be changed in simulation/robot hardware
- Using the State Estimator (and disabling it if you don't want it)
- Using the Dynamics
- Final update rate to legs.
- Are other people going to use the wireless RC controller?  If so, add it to the robot controller.
- Safety checks
- Visualization data - not everything is implemented and they don't work on the robot


# Running on the robot differences
Running on the robot is very similar to running the simulator.  You will still have the gamepad, user parameters, and robot parameters.  In the simulation window, you will see only the state estimate of the robot, and cheater mode will not work.  Currently debugging visualizations don't work.




