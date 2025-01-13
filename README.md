
# **RL24_HW_1**  
Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli  

## **Overview**  
The goal of this homework is to build ROS packages to simulate a 4-degrees-of-freedom robotic manipulator arm in the Gazebo environment.

---

## **Instructions**

1. Clone the repository from GitHub:  
   ```bash
   cd src
   git clone -b REV_3 https://github.com/ferd-bot/RL_24_Homework_1_Robotics.git .
   ```
   **Important**:  
   The `git clone` command with the dot (`.`) works only if the target directory is empty.  
   - If not, you can remove extra files using:
     ```bash
     rm -rf *
     ```
   - Alternatively, clone the repository normally (without the dot) and manually move the files from the `RL_24_Homework_1_Robotics` folder to the `src` directory.

2. Configure and build all packages in the ROS2 workspace:  
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

---

## **Launching the Packages**

1. To launch `arm_description` with Rviz:  
   ```bash
   ros2 launch arm_description display.launch.py
   ```

2. To launch `arm_gazebo` without controllers:  
   ```bash
   ros2 launch arm_gazebo arm_world.launch.py
   ```

3. To launch `arm_gazebo` with controllers:  
   ```bash
   ros2 launch arm_gazebo arm_gazebo.launch.py
   ```
   Once launched this way, it is possible to send position commands to the topic `/position_controller/commands` via terminal:  
   ```bash
   ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, 0.0, 0.3, 0.0]}"
   ```

4. To start the `arm_controller_node`, open another terminal after launching `arm_gazebo.launch.py`:  
   ```bash
   ros2 run arm_control talker_listener
   ```
   In this way, the manipulator will execute a predefined position command (in our case, the position: `[0.5, -0.5, 0.3, -0.3]`), and the current joint positions will be printed in the terminal.

---

## **Notes**

1. You can also launch the `arm_control` file:  
   ```bash
   ros2 launch arm_control arm_control.launch.py
   ```

2. To visualize what the camera is transmitting, open `rqt` in another terminal and connect to the `/videocamera` topic:  
   ```bash
   rqt
   ```

---
