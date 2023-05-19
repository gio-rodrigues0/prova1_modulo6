#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose
from collections import deque
import csv

MAX_DIFF = 0.1

class Pose(TPose):

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        return f"(x={self.x:.2f}, theta={self.y:.2f})"
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF and abs(self.y - other.y) < MAX_DIFF 

class ControlIda(deque):

    def __init__(self, csv_file="./utils/pontos.csv"):
        super().__init__()
        with open(csv_file) as csv_file_var:
            csv_reader = csv.reader(csv_file_var, delimiter=",")
            for row in csv_reader:
                pose = Pose()
                pose.x, pose.y = [float(x) for x in row]
                self.enqueue(pose)

    def enqueue(self, x):
        super().append(x)
    
    def dequeue(self):
        return super().popleft()
    
class ControlVolta(deque):
    def __init__(self):
        super().__init__()

    def push(self, x):
        super().appendleft(x)

    def pop(self):
        return super().pop()

    def __repr__(self):
        return super().__repr__()

class TurtleController(Node):
    def __init__(self, control_ida, control_volta, control_period=0.02):
        super().__init__('turtle_controller')
        self.pose = Pose(x=-10.0)
        self.setpoint = Pose(x=-10.0)
        self.control_ida = control_ida
        self.control_volta = control_volta
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )
        self.control_timer = self.create_timer(
                timer_period_sec=control_period,
                callback=self.control_callback
        )

    def control_callback(self):
        if self.pose.x == -10.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        msg = Twist()
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.5 if y_diff > 0 else -0.5
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        else:
            msg.linear.x = 0.0
        self.publisher.publish(msg)
        
    def update_setpoint(self):
        try:
            self.control_volta.push(self.control_ida[0])
            self.setpoint = self.pose + self.control_ida.dequeue()
            self.get_logger().info(f"Tartaruga chegou em {self.pose}, andando para {self.setpoint}")
        except IndexError:
            try:
                self.setpoint = self.pose + self.control_volta.pop()
                self.get_logger().info(f"Tartaruga chegou em {self.pose}, andando para {self.setpoint}")
                
            except:
                self.get_logger().info(f"Fim da jornada!")
                exit()

    def pose_callback(self, msg):
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == -10.0:
            self.update_setpoint()


def main(args=None):
    rclpy.init(args=args)
    ci = ControlIda()
    cv = ControlVolta
    tc = TurtleController(ci, cv)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()