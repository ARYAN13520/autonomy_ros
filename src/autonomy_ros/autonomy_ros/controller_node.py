import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.get_logger().info("Controller Node Started")

        # --- State ---
        self.drone_alive = False

        # --- Subscribers ---
        self.sub_vel = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.vel_callback,
            10
        )

        self.sub_alt = self.create_subscription(
            Float32,
            '/drone/cmd_alt',
            self.alt_callback,
            10
        )

        self.sub_state = self.create_subscription(
            String,
            '/drone/state',
            self.state_callback,
            10
        )

        # --- Publisher to MAVLink bridge ---
        self.pub_cmd = self.create_publisher(
            Twist,
            '/drone/mav_cmd',
            10
        )

        # Safety limits
        self.MAX_VEL = 2.0
        self.MAX_YAW = 1.0


    # --------------------------------------------------

    def state_callback(self, msg):
        if "Alive" in msg.data:
            self.drone_alive = True


    # --------------------------------------------------

    def vel_callback(self, msg: Twist):

        if not self.drone_alive:
            self.get_logger().warn("Drone not ready - ignoring vel command")
            return

        safe = Twist()

        # Limit velocities
        safe.linear.x = max(min(msg.linear.x, self.MAX_VEL), -self.MAX_VEL)
        safe.linear.y = max(min(msg.linear.y, self.MAX_VEL), -self.MAX_VEL)

        safe.angular.z = max(min(msg.angular.z, self.MAX_YAW), -self.MAX_YAW)

        self.pub_cmd.publish(safe)

        self.get_logger().info(
            f"VEL → x:{safe.linear.x} y:{safe.linear.y} yaw:{safe.angular.z}"
        )


    # --------------------------------------------------

    def alt_callback(self, msg: Float32):

        if not self.drone_alive:
            return

        twist = Twist()

        twist.linear.z = msg.data

        self.pub_cmd.publish(twist)

        self.get_logger().info(f"ALT command → {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

