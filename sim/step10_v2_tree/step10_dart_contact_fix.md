# Step10 DART contact fix

Problem observed:
- Visual cutting-zone mesh rendered correctly.
- DART reported: mesh collision could not be created.
- Therefore no Gazebo contact messages reached ROS2.

Fix:
- Keep the thin yellow mesh as visual only.
- Replace mesh collision with four DART-friendly primitive collisions:
  - top box
  - bottom box
  - rear cylinder
  - front cylinder
- Add one contact sensor per primitive.
- Bridge all four Gazebo contact topics into ROS2.
- TargetContactMonitor subscribes to all four and publishes `/lumberjack_arm/target_contact`.
- A 5 Hz heartbeat publishes false/true continuously for easier debugging.
