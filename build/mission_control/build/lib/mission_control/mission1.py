#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import csv
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Vector3
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommand
import time
import math

class MissionOne(Node):
    def __init__(self):
        super().__init__('mission_one')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/px4_visualizer/vehicle_pose',
            self.position_callback,
            10
        )

        # Publishers
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos_profile
        )

        self.arm_publisher = self.create_publisher(
            Bool,
            '/arm_message',
            qos_profile
        )

        self.vtol_publisher_fw = self.create_publisher(
            Bool,
            '/vtol_message_fw',
            qos_profile
        )

        self.vtol_publisher_mc = self.create_publisher(
            Bool,
            '/vtol_message_mc',
            qos_profile
        )

        # self.commander_publisher = self.create_publisher(
        #     VehicleCommand,
        #     '/fmu/in/vehicle_command',
        #     10
        # )


        # MAIN LOGIC 

        

        self.csv_file_path = self.get_csv_file_path()
        self.csv_data = self.csv_to_mem(self.csv_file_path)

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 8.0},
<<<<<<< Updated upstream
            {'x': 0.0, 'y': -50.0, 'z': 8.0},
            {'x': 80.0, 'y': 0.0, 'z': 4.0},
=======
            {'x': 248.33, 'y': 111.53, 'z': 20.0},
            {'x': -100.17, 'y': 240.18, 'z': 20.0},
            {'x': -31.75, 'y': 96.96, 'z': 10.0},
            {'x': 53.25, 'y': -91.07, 'z': 10.0},
            {'x': 142.13, 'y': -96.41, 'z': 20.0},
            {'x': 217.31, 'y': -162.45, 'z': 20.0},
            {'x': 175.19, 'y': -185.25, 'z': 20.0},
            {'x': 18.82, 'y': -94.74, 'z': 8.0},
>>>>>>> Stashed changes
            {'x': 0.0, 'y': 0.0, 'z': 8.0}
        ]

        self.curr_way_index = 0
<<<<<<< Updated upstream
        self.position_tolerance = 5
=======
        self.position_tolerance = 3.0
        self.s_position_tolerance = 30.0
>>>>>>> Stashed changes
        self.vtol_count = 0
        self.s_waypoint_ind = 0
        self.max_s_waypoint = self.s_way_len(self.csv_data)
        self.curr_height = 0.0

        time.sleep(5)
        self.get_logger().info("Main launched")
        self.arm_drone(True)
        time.sleep(10)
        self.navigate_waypoints()





    def position_callback(self, msg):
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y
        self.current_position['z'] = msg.pose.position.z
<<<<<<< Updated upstream
=======
        self.current_orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
    
    def get_csv_file_path(self):
        package_share_directory = get_package_share_directory('mission_control')
        csv_file_path = os.path.join(package_share_directory, 'test_clothoid_result.csv')
        return csv_file_path

    def csv_to_mem(self, csv_file_path):
        with open(csv_file_path, mode='r') as file:
            csv_reader = list(csv.reader(file))
        return csv_reader
    
    def way_data(self, data, col, curr_height):
        if col < len(data):
            row = data[col]
            if len(row) >= 2:
                x = round(float(row[0]), 2)
                y = round(float(row[1]), 2)
                z = curr_height
                return {'x': x, 'y': y, 'z': z}
        return None
    
    def s_way_len(self, data):
        return len(data)

>>>>>>> Stashed changes

    def arm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone armed")

    def disarm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone disarmed")

    def arm_vtol(self, varm):
        vtol_hmg = Bool()
        vtol_hmg.data = varm
        self.vtol_publisher_fw.publish(vtol_hmg)
        self.get_logger().info("Go VTOL")
    
    def arm_mc(self, marm):
        vtol_mmg = Bool()
        vtol_mmg.data = marm
        self.vtol_publisher_mc.publish(vtol_mmg)
        self.get_logger().info("Fuck. Go back")

    def navigate_waypoints(self):
        self.wp_timer = self.create_timer(0.005, self.navigate_waypoint_callback)

<<<<<<< Updated upstream
=======
    # def navigate_waypoint_callback(self):
    #     if self.curr_way_index < len(self.waypoints):
    #         target = self.waypoints[self.curr_way_index]
    #         if self.is_waypoint_reached(target):
    #             if self.is_waypoint_reached(target) and self.curr_way_index == 1 and self.vtol_count == 1:
    #                 self.arm_mc(True)
    #                 self.vtol_count += 1
    #             elif self.is_waypoint_reached(target) and self.curr_way_index == 2 and self.vtol_count == 3:
    #                 self.arm_mc(True)
    #                 self.vtol_count += 1
    #             self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
    #             self.curr_way_index += 1
    #         elif (self.current_position['y'] < -10.0) and self.curr_way_index == 1 and self.vtol_count == 0:
    #             self.arm_vtol(True)
    #             self.vtol_count += 1
    #         elif (self.current_position['x'] > 10) and self.curr_way_index == 2 and self.vtol_count == 2:
    #             self.arm_vtol(True)
    #             self.vtol_count += 1
    #         else:
    #             twist = self.calculate_velocity_command(target)
    #             self.velocity_publisher.publish(twist)
                

    #     else:
    #         self.get_logger().info("All waypoints reached")
    #         self.disarm_drone(False)
    #         self.destroy_timer(self.wp_timer)

    # def navigate_waypoint_callback(self):
    #     if self.curr_way_index < len(self.waypoints):
    #         target = self.waypoints[self.curr_way_index]
    #         if self.is_waypoint_reached(target):
    #             if self.is_waypoint_reached(target) and self.curr_way_index == 1 and self.vtol_count == 1:
    #                 self.arm_mc(True)
    #                 self.vtol_count += 1
    #             elif self.is_waypoint_reached(target) and self.curr_way_index == 2 and self.vtol_count == 3:
    #                 self.arm_mc(True)
    #                 self.vtol_count += 1
    #             self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
    #             self.curr_way_index += 1
    #         elif (self.current_position['y'] < -10.0) and self.curr_way_index == 1 and self.vtol_count == 0:
    #             self.arm_vtol(True)
    #             self.vtol_count += 1
    #         elif (self.current_position['x'] > 10) and self.curr_way_index == 2 and self.vtol_count == 2:
    #             self.arm_vtol(True)
    #             self.vtol_count += 1
    #         else:
    #             pose = self.calculate_position_command(target)
    #             self.position_publisher.publish(pose)

    #     else:
    #         self.get_logger().info("All waypoints reached")
    #         self.disarm_drone(False)
    #         self.destroy_timer(self.wp_timer)

>>>>>>> Stashed changes
    def navigate_waypoint_callback(self):
        if self.s_waypoint_ind < self.max_s_waypoint:
            target = self.way_data(self.csv_data, self.s_waypoint_ind, self.curr_height)
            m_target = self.waypoints[self.curr_way_index]
            self.curr_height = m_target['z']
            if self.s_is_waypoint_reached(target):
                if self.is_waypoint_reached(m_target):
                    self.curr_way_index += 1
                    self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
                self.s_waypoint_ind += 1
            elif (self.current_position['x'] > 25.0) and self.curr_way_index == 1 and self.vtol_count == 0:
                self.arm_vtol(True)
                self.vtol_count += 1
            else:
                twist = self.calculate_velocity_command(target)
                self.velocity_publisher.publish(twist)
                

        else:
            self.get_logger().info("All waypoints reached")
            self.disarm_drone(False)
            self.destroy_timer(self.wp_timer)

    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 0.2
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y
        twist.linear.z = kp * error_z

        max_speed = 1.0
        norm = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if norm > max_speed:
            twist.linear.x = (twist.linear.x / norm) * max_speed
            twist.linear.y = (twist.linear.y / norm) * max_speed
            twist.linear.z = (twist.linear.z / norm) * max_speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        return twist

    def is_waypoint_reached(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)
    
    def s_is_waypoint_reached(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        return (x_dist < self.s_position_tolerance and
                y_dist < self.s_position_tolerance and
                z_dist < self.s_position_tolerance)
    


def main(args=None):
    rclpy.init(args=args)
    mission1node = MissionOne()
    rclpy.spin(mission1node)
    mission1node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()