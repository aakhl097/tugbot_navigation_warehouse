#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import sys, select
import subprocess

class Lidar_Topic_Converter(Node):
    def __init__(self):
        super().__init__("lidar_gazebo_ros_bridge")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.scan_publisher = self.create_publisher(LaserScan, 'lidar_scan', qos)
        command = ["gz", "topic", "--echo", "--topic", "/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan"]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.timer = self.create_timer(1/30, self.timer_callback)
    
    def timer_callback(self):
        all_distances = []
        phi = []
        i=0
  
        while select.select([self.process.stdout], [], [], 0)[0]:
            
            line = self.process.stdout.readline().decode().strip()
            if line.startswith("ranges:"):
            
                for v in line.split(":",1)[1].split():
                    distance = float(v)
                    bearing = (i*0.0043698) -1.47
                    phi.append(bearing)
                    all_distances.append(distance)
                    #print(i)
                    #print(all_distances[i])
                    i=i+1
                    #print("\n")
                
                if len(all_distances) >= 650:
                    laser_message = LaserScan()
                    laser_message.header.frame_id = 'lidar_link'
                    laser_message.angle_min = -1.47043991089
                    laser_message.angle_max = 1.47043991089
                    laser_message.angle_increment = 0.0043698065702526 #(laser_message.angle_max - laser_message.angle_min) / len(all_distances)
                    laser_message.range_min = 0.01
                    laser_message.range_max = 5.0
                    laser_message.ranges = all_distances

                    #print(all_distances[300])
                    #print(phi[300])
                    #print("\n")
                
                    #if len(phi) > 380:
                    #    print(phi[379])
                    #    print("d")
                    #    print(all_distances[379])
                    self.scan_publisher.publish(laser_message)
                    phi.clear()
                    all_distances.clear()
                    i=0

def main(args = None):
    rclpy.init(args=args)
    node = Lidar_Topic_Converter()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()