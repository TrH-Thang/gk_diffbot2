This GK_diffbot for ROS2 Simulation

## Setup

### 1. Clone Repository
Clone the gk_diffbot repositories using the following commands:

```bash
cd ~/catkin_ws/src
git clone https://github.com/TrH-Thang/gk_diffbot2.git
colcon build
```

### 2. Launch Simulation

```bash
cd ~/catkin_ws
source install/setup.bash
ros2 launch gk_diffbot gk_sim.launch.py 
```

open a new terminal:
```bash
cd ~/catkin_ws
source install/setup.bash
ros2 run teleop_control teleop_keyboard   # This node for robot control
```

open a new terminal and enter:
```bash
cd ~/catkin_ws
source install/setup.bash
ros2 run teleop_control arm_keyboard    # This node for arm control
```
