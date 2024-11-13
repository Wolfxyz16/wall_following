#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math as m
from tf_transformations import euler_from_quaternion

import csv
from sklearn.linear_model import LinearRegression
import time
from rclpy.signals import SignalHandlerOptions 

class Robot(Node):
    def __init__(self):
        super().__init__('wall_following_node')
        # kp value to use in the execution
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.5),
                ('vel_topic', 'cmd_vel'),
                ('odom_topic', 'odom'),
                ('output_filename', 'wf_ls')
            ]
        )
        self.kp = self.get_parameter('kp').value
        
        # velocity topic
        self.vel_topic = self.get_parameter('vel_topic').value
        # odometry topic
        self.odom_topic = self.get_parameter('odom_topic').value

        ''' output filename (with no extension). 
        kp value and .csv extension will be added 
        to this name afterwards 
        '''
        f = self.get_parameter('output_filename').value
        fout = f + '_' + str(self.kp) + '.csv'
        file = open(fout, 'w')
        self.csvwriter = csv.writer(file, delimiter = ',')
        self.csvwriter.writerow(['kp', 'error', 'v', 'w', 'x', 'y'])        
        
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.follow_wall, 1)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.get_position, 1)
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 1)
        
        self.scan_count = 0
        self.range_min = 0.0
        self.max_range = 8.0
        self.bearings = []
        self.rx = 0
        self.ry = 0
        self.rtheta = 0
        self.uninitialized = True
        
    def get_position(self, msg):
        # Gets robot pose (x, y, yaw) from odometry
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.rtheta = yaw

    def follow_wall(self, scan):             
        lscan = scan.ranges
        cmd_vel_msg = Twist()
        if self.uninitialized:
            self.uninitialized = False
            self.scan_count = len(scan.ranges)
            self.range_min = scan.range_min
            for i in range(self.scan_count):
                self.bearings.append(scan.angle_min + scan.angle_increment * i)
        
        self.scan = [scan.ranges[i - 800] for i in range(self.scan_count)]
        
        ## TODO Add code here


        # 1.- Compute the projection of short readings 
        xpos = np.empty((self.scan_count, 1), float)
        ypos = np.empty((self.scan_count, 1), float)

        

        # Select the short readings and calculate the corresponding points
        # Cogemos las lecturas del cuadrante superior derecha (400-800, ya que el 0 está atrás)
        


        for i in range(400,800):
            if self.scan[i] < 2.3:
                xpos[i] = m.cos(self.bearings[i] + (m.pi)) * self.scan[i]
                ypos[i] = m.sin(self.bearings[i] + (m.pi)) * self.scan[i]


        j = min(200,len(xpos))

        # 2.- Linear regression: compute the line from the selected points   
        c1 = 0.0
        c0 = 0.0
        theta2 = 0.0
        if j > 5: 
            # Note: The j index indicates the number of points used in the regression!!
            xpos.resize(j, 1)
            ypos.resize(j, 1)

            model = LinearRegression()
            model.fit(xpos, ypos)
            c0 = float(model.intercept_)
            c1 = float(model.coef_)

            self.get_logger().info(f"y = {c1:.2f}x + {c0:.2f}")

	        # Calculate the angle between the robot and the wall
            theta = 0.0
            #self.get_logger().info("Line slope: %.2f Angle: %.2f (%.2f degrees)"%(c1, theta, m.degrees(theta)))
            
            # 3.- Set velocities proportionally
            w = self.kp * theta
            v = 0.0
            #self.get_logger().info("Velocities: v = %.2f w = %.2f"%(v, w))

            self.get_logger().info(f"{self.scan[400]}")
            self.get_logger().info(f"{self.scan[600]}")
            self.get_logger().info(f"{self.scan[799]}")


            ## END TODO

            cmd_vel_msg.linear.x  = v
            cmd_vel_msg.linear.y  = 0.0
            cmd_vel_msg.angular.z = w

        else:
            self.get_logger().info("Not enough points for applying the linear regression")
            cmd_vel_msg.linear.x  = 0.2
            cmd_vel_msg.linear.y  = 0.0
            cmd_vel_msg.angular.z = 0.0

        self.csvwriter.writerow([f"{self.kp:.2f}", f"{c0:.2f}", f"{c1:.2f}", f"{theta2:.2f}", f"{cmd_vel_msg.linear.x:.2f}", f"{cmd_vel_msg.angular.z:.2f}", f"{self.rx:.2f}", f"{self.ry:.2f}"])    
        self.vel_pub.publish( cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    wf_node = Robot()
    try:
        rclpy.spin(wf_node)
    except KeyboardInterrupt:
        wf_node.destroy_node()

if __name__ == '__main__':
    main()
