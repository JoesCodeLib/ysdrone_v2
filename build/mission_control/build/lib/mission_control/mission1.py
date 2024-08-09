#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
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

        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/offboard_position_cmd',
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

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 8.0},
            {'x': 0.0, 'y': -50.0, 'z': 8.0},
            {'x': 50.0, 'y': 0.0, 'z': 8.0},
            {'x': 0.0, 'y': 0.0, 'z': 8.0}
        ]

        self.curr_way_index = 0
        self.position_tolerance = 2.0
        self.vtol_count = 0

        time.sleep(5)
        self.get_logger().info("Main launched")
        self.arm_drone(True)
        time.sleep(10)
        self.navigate_waypoints()





    def position_callback(self, msg):
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y
        self.current_position['z'] = msg.pose.position.z
        self.current_orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

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
        self.wp_timer = self.create_timer(0.01, self.navigate_waypoint_callback)

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

    def navigate_waypoint_callback(self):
        if self.curr_way_index < len(self.waypoints):
            target = self.waypoints[self.curr_way_index]
            if self.is_waypoint_reached(target):
                if self.is_waypoint_reached(target) and self.curr_way_index == 1 and self.vtol_count == 1:
                    self.arm_mc(True)
                    self.vtol_count += 1
                elif self.is_waypoint_reached(target) and self.curr_way_index == 2 and self.vtol_count == 3:
                    self.arm_mc(True)
                    self.vtol_count += 1
                self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
                self.curr_way_index += 1
            elif (self.current_position['y'] < -10.0) and self.curr_way_index == 1 and self.vtol_count == 0:
                self.arm_vtol(True)
                self.vtol_count += 1
            elif (self.current_position['x'] > 10) and self.curr_way_index == 2 and self.vtol_count == 2:
                self.arm_vtol(True)
                self.vtol_count += 1
            else:
                pose = self.calculate_position_command(target)
                self.position_publisher.publish(pose)

        else:
            self.get_logger().info("All waypoints reached")
            self.disarm_drone(False)
            self.destroy_timer(self.wp_timer)


    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 1.4
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

        # Calculate desired yaw
        desired_yaw = math.atan2(error_y, error_x)
        current_yaw = self.get_current_yaw()  # Assuming you have a method to get current yaw
        error_yaw = desired_yaw - current_yaw

        # Normalize yaw error to range [-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        kp_yaw = 0.1
        twist.angular.z = kp_yaw * error_yaw

        return twist
    
    def calculate_position_command(self, target):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['x']
        pose.pose.position.y = target['y']
        pose.pose.position.z = target['z']

        # Calculate desired yaw
        error_y = target['y'] - self.current_position['y']
        error_x = target['x'] - self.current_position['x']
        desired_yaw = math.atan2(error_y, error_x)

        # Set yaw in Euler angles directly
        pose.pose.orientation = self.euler_to_quaternion(0, 0, desired_yaw)

        return pose

    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        roll, pitch, yaw: in radians
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw

        return quaternion

    
    def get_current_yaw(self):
        # Assuming you have a method to get the current orientation as a quaternion
        orientation = self.current_orientation
        _, _, yaw = self.euler_from_quaternion(orientation)
        return yaw

    def euler_from_quaternion(self, quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        quat: [x, y, z, w]
        """
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


    def is_waypoint_reached(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)

def main(args=None):
    rclpy.init(args=args)
    mission1node = MissionOne()
    rclpy.spin(mission1node)
    mission1node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()