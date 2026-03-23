import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class FlightService(Node):

    def __init__(self):
        super().__init__('flight_service')

        # Publisher to existing controller
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Services
        self.create_service(Trigger, 'takeoff', self.takeoff_cb)
        self.create_service(Trigger, 'land', self.land_cb)
        self.create_service(Trigger, 'stop', self.stop_cb)
        self.create_service(Trigger, 'up2m', self.up2m_cb)

        self.get_logger().info("Flight Services Ready")


    # -------------------------------

    def send(self, x=0.0, z=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.z = z
        msg.angular.z = yaw
        self.pub.publish(msg)

    # -------------------------------

    def takeoff_cb(self, req, res):
        self.get_logger().info("TAKEOFF 2m")

        # climb
        self.send(z=-0.6)

        res.success = True
        res.message = "Takeoff command sent"
        return res

    # -------------------------------

    def land_cb(self, req, res):
        self.get_logger().info("LAND")

        # down
        self.send(z=0.5)

        res.success = True
        res.message = "Landing command sent"
        return res

    # -------------------------------

    def stop_cb(self, req, res):
        self.get_logger().info("STOP ALL")

        self.send(0,0,0)

        res.success = True
        res.message = "Stop sent"
        return res

    # -------------------------------

    def up2m_cb(self, req, res):
        self.get_logger().info("UP small step")

        self.send(z=-0.3)

        res.success = True
        res.message = "Up step sent"
        return res



def main(args=None):
    rclpy.init(args=args)
    node = FlightService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

