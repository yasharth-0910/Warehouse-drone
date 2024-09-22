#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Publisher to control the turtle's velocity
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Clients for teleporting and setting pen
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute service...')

        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        # Set pen properties (optional)
        self.set_pen(255, 255, 255, 2, 0)  # White color, thickness 2, off (not erased)

    def set_pen(self, r, g, b, width, off):
        """Set the pen properties."""
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = off

        future = self.pen_client.call_async(pen_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Set pen to RGB({r}, {g}, {b}) with width {width}')
        else:
            self.get_logger().error('Failed to set pen')

    def teleport(self, x, y, theta):
        """Teleport the turtle to a specific (x, y) position with orientation theta."""
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = x
        teleport_request.y = y
        teleport_request.theta = theta

        future = self.teleport_client.call_async(teleport_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Teleported to ({x}, {y}, {theta})')
        else:
            self.get_logger().error('Failed to teleport')

    def draw_circle(self, radius=1.0, speed=1.0):
        """Draw a circle with the given radius and speed."""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius  # v = r * omega => omega = v / r

        duration = 2 * math.pi * radius / speed  # Time to complete the circle
        self.get_logger().info(f'Drawing circle: radius={radius}, duration={duration:.2f}s')

        start_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        end_time = start_time + duration

        while True:
            current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
            if current_time >= end_time:
                break
            self.publisher_.publish(twist)
            time.sleep(0.1)  # Publish at 10 Hz

        # Stop the turtle after drawing
        self.stop_turtle()

    def move_straight(self, distance=2.0, speed=2.0):
        """Move the turtle straight for a certain distance at a given speed."""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        duration = distance / speed  # Time to move the distance
        self.get_logger().info(f'Moving straight: distance={distance}, duration={duration:.2f}s')

        start_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        end_time = start_time + duration

        while True:
            current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
            if current_time >= end_time:
                break
            self.publisher_.publish(twist)
            time.sleep(0.1)  # Publish at 10 Hz

        # Stop the turtle after moving
        self.stop_turtle()

    def stop_turtle(self):
        """Stop the turtle's movement."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)  # Ensure the turtle stops

    def execute(self):
        """Execute the drawing sequence."""
        # 1. Draw Four Circles (Propellers)
        propellers = [(2.0, 2.0), (2.0, 8.0), (8.0, 8.0), (8.0, 2.0)]
        for idx, (x, y) in enumerate(propellers, start=1):
            self.teleport(x, y, 0.0)  # Teleport to propeller center
            self.draw_circle(radius=1.0, speed=1.0)  # Draw circle with radius 1.0
            self.get_logger().info(f'Completed drawing propeller {idx}')
            time.sleep(1)

        # 2. Draw the Square Frame
        square_vertices = [(3.0, 5.0), (5.0, 7.0), (7.0, 5.0), (5.0, 3.0)]
        self.teleport(*square_vertices[0], 0.0)  # Teleport to first vertex
        self.get_logger().info('Starting to draw the square frame')

        for idx, vertex in enumerate(square_vertices):
            next_vertex = square_vertices[(idx + 1) % len(square_vertices)]
            self.move_to_point(vertex, next_vertex, 2.0)  # Move turtle from current to next vertex
            self.get_logger().info(f'Drew line from {vertex} to {next_vertex}')
            time.sleep(1)

        # 3. Draw Connecting Lines to Complete the Frame
        midpoints = [(4.0, 6.0), (6.0, 6.0), (6.0, 4.0), (4.0, 4.0)]
        self.get_logger().info('Starting to draw connecting lines')
        for idx, (x, y) in enumerate(midpoints, start=1):
            self.teleport(x, y, 0.0)  # Teleport to the midpoint
            self.move_straight(distance=2.0, speed=2.0)  # Draw line to center
            self.get_logger().info(f'Drew connecting line {idx} to center')
            time.sleep(1)

        # 4. Move Turtle to the Center
        self.teleport(5.0, 5.0, 0.0)
        self.get_logger().info('Turtle returned to the center (5.0, 5.0)')



def main(args=None):
    print("Starting Task 1A...")  # Task Start Message

    rclpy.init(args=args)

    turtle_controller = TurtleController()

    # Give some time for setup
    time.sleep(2)

    try:
        turtle_controller.execute()
    except KeyboardInterrupt:
        turtle_controller.get_logger().info('Interrupted by user')
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
