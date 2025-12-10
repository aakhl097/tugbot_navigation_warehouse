#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
import subprocess
import sys, select

class Orientation_Converter(Node):
    def __init__(self):
        super().__init__("orientation_topic_converter")
        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        command = ['gz', 'topic', '--echo', '--topic', '/model/tugbot/pose']
        self.process = subprocess.Popen(command, stdout = subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.orientation_publisher = self.create_publisher(Pose, '/quaternion_parameters', qos)
        self.timer = self.create_timer(1/30, self.timer_callback)
        self.message_to_send = Pose()
        self.message = ""
        self.orientation_message = ""
        self.capture = False
        self.capture_orientation = False

    def timer_callback(self):
        while select.select([self.process.stdout], [], [], 0)[0]:
            line = self.process.stdout.readline().decode().strip()
            if line.startswith('value: "world_demo"'):
                self.message = line + "\n"
                self.capture = True
                
            elif self.capture == True:
                self.message += line + "\n"
                
                if line.startswith("orientation"):
                    self.capture_orientation = True
            
                if self.capture_orientation==True:
                    self.orientation_message += line + "\n"

                    if line.startswith("w:"):
                        self.capture = False
                        self.capture_orientation = False
                        #print(self.orientation_message)
                        
                        orientation_lines = self.orientation_message.strip().split("\n")
                        for mylines in orientation_lines:
                            l = mylines.strip()
                            if l.startswith('x'):
                                self.message_to_send.orientation.x = float(l.split(":", 1)[1])
                            if l.startswith('y'):
                                self.message_to_send.orientation.y = float(l.split(":", 1)[1])
                            if l.startswith('z'):
                                self.message_to_send.orientation.z = float(l.split(":", 1)[1])
                            if l.startswith('w'):
                                self.message_to_send.orientation.w = float(l.split(":", 1)[1])
                            self.orientation_publisher.publish(self.message_to_send)
                            self.orientation_message=""
                            self.message = ""

def main(args = None):
    rclpy.init(args=args)
    node = Orientation_Converter()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 