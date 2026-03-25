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

        self.armed = False
        self.mode = 0
        self.airborne = False   # 🔥 IMPORTANT

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

        self.create_timer(1.0, self.publish_state)
        self.create_timer(1.0, self.check_vehicle_state)

    # -------------------------------------------------------

    def check_vehicle_state(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)

        if msg:
            self.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.mode = msg.custom_mode

    # -------------------------------------------------------

    def publish_state(self):
        msg = String()
        msg.data = "Drone Alive"
        self.pub.publish(msg)

    # -------------------------------------------------------

    def set_guided_and_arm(self):

        for _ in range(5):
            self.check_vehicle_state()
            time.sleep(0.1)

        if not self.armed:
            self.get_logger().warn("Drone not armed → trying to arm")

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )

            for _ in range(20):
                self.check_vehicle_state()
                if self.armed:
                    self.get_logger().info("Drone ARMED confirmed")
                    break
                time.sleep(0.2)

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4
        )

        time.sleep(0.5)

        self.check_vehicle_state()

        if not self.armed:
            self.get_logger().warn("Drone STILL not armed")
            return False

        return True

    # -------------------------------------------------------

    def takeoff(self, altitude=3.0):

        self.get_logger().info(f"Initiating TAKEOFF to {altitude}m")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, altitude
        )

        self.get_logger().info("Waiting for takeoff...")
        time.sleep(5)

    # -------------------------------------------------------

    def send_yaw(self, yaw_rate):

        target_angle = yaw_rate * 45.0
        speed = 20

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            target_angle,
            speed,
            1,
            1,
            0, 0, 0
        )

    # -------------------------------------------------------

    def send_velocity(self, vx, vz):

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, 0, vz,
            0, 0, 0,
            0, 0
        )

    # -------------------------------------------------------

    def cmd_callback(self, msg: Twist):

        # Ensure ready
        if not self.set_guided_and_arm():
            return

        # 🔥 TAKEOFF FIRST
        if not self.airborne:
            self.takeoff(3.0)
            self.airborne = True
            return

        yaw = msg.angular.z
        forward = msg.linear.x
        climb = msg.linear.z

        # STOP
        if abs(yaw) < 0.05 and abs(forward) < 0.05 and abs(climb) < 0.05:
            self.get_logger().info("STOP command received")
            self.send_yaw(0.0)
            self.send_velocity(0.0, 0.0)
            return

        # YAW
        if abs(yaw) > 0.05:
            self.send_yaw(yaw)
            self.get_logger().info(f"Yaw command sent: {yaw}")

        # VELOCITY
        if abs(forward) > 0.05 or abs(climb) > 0.05:
            self.send_velocity(forward, climb)
            self.get_logger().info(
                f"Vel cmd: fwd={forward}, climb={climb}"
            )


# -----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = MAVLinkNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
