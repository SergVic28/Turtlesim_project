#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sqrt, atan2, pi
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle_first", True)

        self.turtle_pose_ = None
        self.turtle_to_catch_ = None
        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first").value
        # self.target_ = [2.0, 5.0]  # x, y

        self.turtle_command_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.subscriber_turtle_pose_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Turtle Controller node has been started.")

    def callback_turtle_pose(self, msg):
        self.turtle_pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.turtle_pose_.x
                    dist_y = turtle.y - self.turtle_pose_.y
                    distance = sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.turtle_pose_ == None or self.turtle_to_catch_ == None:
            return

        dist_x = self.turtle_to_catch_.x - self.turtle_pose_.x
        dist_y = self.turtle_to_catch_.y - self.turtle_pose_.y
        distance = sqrt(dist_x**2 + dist_y**2)

        move_cmd = Twist()

        if distance > 0.5:
            self.get_logger().info("Turtle on the way")

            # self.cos_alpha = (self.turtle_pose_.x * self.target_[0] + self.turtle_pose_.y * self.target_[1]) / (
            #     sqrt(self.turtle_pose_.x**2 + self.turtle_pose_.y**2) * sqrt(self.target_[0]**2 + self.target_[1]**2))
            # self.alpha = acos(self.cos_alpha)
            # self.degree = self.alpha * 180 / pi
            # move_cmd.linear.x = self.turtle_pose_.x + distance * cos(self.alpha)
            # move_cmd.linear.y = self.turtle_pose_.y + distance * sin(self.alpha)
            move_cmd.linear.x = 2 * distance

            # tan_alpha = (self.target_[1] - self.turtle_pose_.y) / (self.target_[0] - self.turtle_pose_.x)
            theta = atan2(dist_y, dist_x)
            diff = theta - self.turtle_pose_.theta
            if diff > pi:
                diff -= 2 * pi
            elif diff < -pi:
                diff += 2 * pi
            move_cmd.angular.z = 5 * diff

            self.get_logger().info("x = " + str(self.turtle_pose_.x))
            self.get_logger().info("y = " + str(self.turtle_pose_.y))
            self.get_logger().info("theta = " + str(theta))
            self.get_logger().info("diff = " + str(diff))
            self.get_logger().info("distance = " + str(distance))
            # self.get_logger().info("alpha = " + str(self.alpha))
            # self.get_logger().info("degree = " + str(self.degree))

            # distance = sqrt((self.target_[0] - self.turtle_pose_.x)**2
            #                 + (self.target_[1] - self.turtle_pose_.y)**2)
        else:
            # Target reached!
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.turtle_command_publisher_.publish(move_cmd)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
