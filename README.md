🚀 autonomy_ros

ROS2 – ArduPilot SITL Autonomy Bridge

A modular ROS2 package that bridges ArduPilot SITL (Copter) with ROS2 control commands using MAVLink via pymavlink.
This project enables:

• ROS → MAVLink control
• Yaw rotation
• Body-frame velocity
• Basic flight services
• Mission Planner parallel connection.

🧠 Project Goal

To create a clean autonomy layer where:

ROS2 → publishes high-level commands
MAVLink → executes on ArduPilot SITL
Mission Planner → remains connected for monitoring.

This is the foundation for:

• waypoint missions
• vision guidance
• swarm logic
• AI navigation.

📁 Package Structure
autonomy_ros/
│
├── autonomy_ros/
│   ├── mavlink_node.py      → MAVLink <-> ROS bridge
│   ├── controller_node.py   → Velocity command processor
│   └── flight_service.py    → Takeoff / landing services
│
├── launch/
│   └── full.launch.py       → Launches full stack
│
├── package.xml
├── setup.py
└── README.md

✅ Current Features
1. MAVLink Bridge

• Connects to SITL on UDP 14550
• Reads heartbeat
• Arms vehicle
• Forces GUIDED mode
• Sends:

Yaw commands

Body velocity (forward + altitude).

2. ROS Interfaces
Topics

/drone/cmd_vel → geometry_msgs/Twist
Used for:

• angular.z → yaw
• linear.x → forward
• linear.z → altitude.

/drone/state → std_msgs/String
Heartbeat monitor.

Services

/takeoff → std_srvs/Trigger
Initiates takeoff sequence.

3. Mission Planner Integration

• SITL outputs dual UDP
• Mission Planner can stay connected
• ROS can control simultaneously.

🛠 How to Run
1. Start SITL
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14560

2. Launch ROS Stack
source ~/projects/autonomy_ros/install/setup.bash
ros2 launch autonomy_ros full.launch.py

3. Test Commands
Takeoff
ros2 service call /takeoff std_srvs/Trigger

Yaw
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

Forward
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

Up
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{linear: {z: -0.5}}"

Stop
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{}"

⚠ Current Limitations

Takeoff via velocity not reliable

Arming occurs but altitude hold not triggered

Requires:

• pre-armed state from MP
• GUIDED mode externally.

🧩 Next Development Plan
Phase A – Fix Takeoff Logic

• Replace velocity takeoff with
MAV_CMD_NAV_TAKEOFF
• Add altitude monitor
• add EKF ready check.

Phase B – Telemetry

• /drone/pose
• /drone/battery
• /drone/status.

Phase C – Mission Layer

• waypoint executor
• RTL
• failsafe.

Dependencies

• ROS2 Humble
• pymavlink
• ArduPilot SITL
• Mission Planner (optional).

Author

Aryan Hajare
Vice President – Technology
UAV Autonomy Research

License

MIT

Contribution

This is an active research project.
Pull requests welcome after Phase A stabilization.
