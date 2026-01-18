import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import threading
import time


class MavlinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')

        self.get_logger().info("Starting MAVLink Bridge...")

        # ROS Publishers
        self.state_pub = self.create_publisher(String, '/drone/state', 10)

        # ROS Subscriber for control
        self.cmd_sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_callback,
            10
        )

        # Connect to SITL
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()

        self.get_logger().info("Connected to SITL!")

        # Start listener thread
        threading.Thread(target=self.listen_loop, daemon=True).start()

    def listen_loop(self):
        while rclpy.ok():
            msg = self.master.recv_match(blocking=True, timeout=1)

            if msg and msg.get_type() == 'HEARTBEAT':
                ros_msg = String()
                ros_msg.data = "Drone Alive"
                self.state_pub.publish(ros_msg)

    def cmd_callback(self, msg: Twist):
        yaw_rate = msg.angular.z

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, yaw_rate
        )

        self.get_logger().info(f"Yaw command sent: {yaw_rate}")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
