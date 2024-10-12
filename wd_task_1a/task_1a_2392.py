import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi

class DroneTurtleSim(Node):
    def __init__(self):
        super().__init__('drone_turtle_sim')
        self.pen_srv = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_srv = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.draw_drone()

    def set_pen(self, r, g, b, width, off):
        """ Set the pen color, width, and toggle on/off """
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_srv.call_async(req)

    def teleport(self, x, y, theta):
        """ Teleport turtle to specific coordinates without drawing """
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_srv.call_async(req)
        self.get_logger().info(f"Teleporting to: x={x}, y={y}, theta={theta}")

    def draw_circle(self, center_x, center_y, radius, speed):
        """ Draw a circle with the given center and radius """
        # Offset by radius to start from the top of the circle
        start_x = center_x
        start_y = center_y - radius
        
        # Teleport to the starting point
        self.set_pen(0, 0, 0, 3, 1)  # Turn off pen
        self.teleport(start_x, start_y, 0.0)
        
        self.set_pen(255, 255, 255, 3, 0)  # Turn on pen
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius  # omega = linear speed / radius

        total_angle = 2 * pi  # Complete one full circle
        t_start = self.get_clock().now()
        time_needed = total_angle / twist.angular.z  # Time to complete the circle

        self.get_logger().info(f"Starting to draw circle with center ({center_x}, {center_y}) and radius {radius}")

        # Publish twist messages to move the turtle in a circle
        while (self.get_clock().now() - t_start).nanoseconds / 1e9 < time_needed:
            self.cmd_vel_pub.publish(twist)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def draw_line(self, x_start, y_start, x_end, y_end):
        """ Draw a line from (x_start, y_start) to (x_end, y_end) using cmd_vel """
        self.set_pen(255, 255, 255, 3, 1)  # Turn off pen for teleport
        self.teleport(x_start, y_start, 0.0)  # Teleport to start position
        self.set_pen(255, 255, 255, 3, 0)  # Turn on pen for drawing

        # Calculate the distance and angle to move in a straight line
        delta_x = x_end - x_start
        delta_y = y_end - y_start
        distance = sqrt(delta_x**2 + delta_y**2)
        angle = atan2(delta_y, delta_x)

        # Set up velocity commands for linear motion
        twist = Twist()
        twist.linear.x = 1.0  # Set a constant speed for straight motion
        twist.angular.z = 0.0  # No rotation needed

        # First, rotate to the correct angle
        self.set_pen(255, 255, 255, 3, 1)  # Turn off pen to rotate
        self.teleport(x_start, y_start, angle)  # Teleport to start, facing the right direction
        self.set_pen(255, 255, 255, 3, 0)  # Turn on pen again for drawing

        # Calculate time to move the required distance
        time_needed = distance / twist.linear.x
        t_start = self.get_clock().now()

        # Publish velocity commands to move in a straight line for the calculated time
        while (self.get_clock().now() - t_start).nanoseconds / 1e9 < time_needed:
            self.cmd_vel_pub.publish(twist)

        # Stop the turtle after reaching the endpoint
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f"Drawing line from ({x_start}, {y_start}) to ({x_end}, {y_end})")

    def draw_drone(self):
        # Drawing the drone propellers (circles)
        self.draw_circle(2.0, 2.0, 1.0, 1.0)
        self.draw_circle(2.0, 8.0, 1.0, 1.0)
        self.draw_circle(8.0, 8.0, 1.0, 1.0)
        self.draw_circle(8.0, 2.0, 1.0, 1.0)

        # Drawing the square with the given vertices
        self.draw_line(3.0, 5.0, 5.0, 7.0)
        self.draw_line(5.0, 7.0, 7.0, 5.0)
        self.draw_line(7.0, 5.0, 5.0, 3.0)
        self.draw_line(5.0, 3.0, 3.0, 5.0)

        # Drawing the connecting lines
        self.draw_line(2.0, 2.0, 4.0, 4.0)
        self.draw_line(2.0, 8.0, 4.0, 6.0)
        self.draw_line(8.0, 8.0, 6.0, 6.0)
        self.draw_line(8.0, 2.0, 6.0, 4.0)

        # Teleport turtle to the center of the square
        self.set_pen(0, 0, 0, 3, 1)
        self.move_to(5.0, 5.0)

    def move_to(self, x, y):
        """ Teleport turtle to a specific point """
        self.teleport(x, y, 0.0)
        self.get_logger().info(f"Teleported to: x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneTurtleSim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
