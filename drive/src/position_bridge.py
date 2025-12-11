#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
import subprocess
import sys, select
import string

class Topic_Converter(Node):
    def __init__(self):
        super().__init__("position_topic_converter")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.position_publisher = self.create_publisher(Pose, "my_tugbot_pose", qos)
        self.capture = False
        self.poses = Pose()
        command = ["gz", "topic", "--echo", "--topic", "/model/tugbot/pose"]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        while select.select([self.process.stdout], [], [], 0)[0]:
            line = self.process.stdout.readline().decode().strip()
            if line.startswith('value: "world_demo"'):
                self.capture = True
                self.message = line + "\n"
                #print(self.message.strip())
            elif self.capture == True:
                self.message += line + "\n"
                
                if line.startswith('orientation'):
                    self.capture = False
                    #print(self.message.strip())
                    filtered_line = self.message.strip().split("\n")
                    for thelines in filtered_line:
                        l = thelines.strip()
                        if l.startswith('x:'):
                            self.poses.position.x = float(l.split(":", 1)[1])
                        if l.startswith('y:'):
                            self.poses.position.y = float(l.split(":", 1)[1])
                        
                        self.position_publisher.publish(self.poses)
            #if line.startswith('ranges:'):

def main(args=None):
    rclpy.init(args=args)
    node = Topic_Converter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()