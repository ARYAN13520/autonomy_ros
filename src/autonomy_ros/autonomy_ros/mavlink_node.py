import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pymavlink import mavutil
import time


class MAVLinkNode(Node):

    def __init__(self):
        super().__init__('mavlink_node')

        self.get_logger().info("Starting MAVLink Bridge...")

        # Connect to SITL
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

        self.master.wait_heartbeat()
        self.get_logger().info("Connected to SITL!")

        # ROS interfaces
        self.sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/drone/state',
            10
        )

        # heartbeat timer
        self.create_timer(1.0, self.publish_state)

    # -------------------------------------------------------

    def publish_state(self):
        msg = String()
        msg.data = "Drone Alive"
        self.pub.publish(msg)

    # -------------------------------------------------------

    def set_guided_and_arm(self):
        """
        Ensure drone is in GUIDED mode and armed
        """

        # Set GUIDED mode
        self.master.set_mode('GUIDED')
        time.sleep(1)

        # Arm
        self.master.arducopter_arm()
        time.sleep(2)

    # -------------------------------------------------------

    def send_yaw(self, yaw_rate):
        """
        CORRECT ArduPilot yaw command
        Converts angular.z → degrees step
        """

        target_angle = yaw_rate * 45.0   # scale input to degrees
        speed = 20                       # deg/sec

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_CONDITION_YAW,

            0,              # confirmation
            target_angle,   # PARAM1 → TARGET ANGLE
            speed,          # PARAM2 → SPEED
            1,              # PARAM3 → CLOCKWISE
            1,              # PARAM4 → RELATIVE
            0, 0, 0
        )

    # -------------------------------------------------------

    def cmd_callback(self, msg: Twist):
        yaw = msg.angular.z

        self.set_guided_and_arm()

    # --- STOP CONDITION ---
        if abs(yaw) < 0.05:
            self.get_logger().info("Yaw STOP command received")
            self.send_yaw(0.0)
            return

        self.send_yaw(yaw)

        self.get_logger().info(f"Yaw command sent: {yaw}")


# -----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = MAVLinkNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
