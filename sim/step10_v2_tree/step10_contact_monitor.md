# Step10 - Target Branch Contact Monitor

## Goal

Bridge the Gazebo cutting-zone contact sensor into ROS 2 and recognize only contact with the detachable red target branch.

## Gazebo source topic

```text
/world/x500_lumberjack_world/model/x500_lumberjack/link/chainsaw_body/sensor/cutting_zone_contact_sensor/contact
```

## ROS 2 topics

Input contact stream:

```text
/lumberjack_arm/cutting_zone/contacts
```

Filtered boolean state:

```text
/lumberjack_arm/target_contact
```

## Target collision

```text
target_branch::target_branch_link::target_branch_outer_collision
```

## Behavior

- Yellow cutting zone contacts the red target branch -> `CONTACT TRUE`
- Contact disappears for 0.20 s -> `CONTACT FALSE`
- Contact with other objects is ignored
- No saw-speed condition and no detach command are added in this step

## Build

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_lumberjack_control --symlink-install
source install/setup.bash
```

## Run

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_arm_v2.launch.py
```

## Verify bridge

```bash
ros2 topic list | grep -E 'cutting_zone|target_contact'
```

Expected:

```text
/lumberjack_arm/cutting_zone/contacts
/lumberjack_arm/target_contact
```

Watch filtered state:

```bash
ros2 topic echo /lumberjack_arm/target_contact
```
