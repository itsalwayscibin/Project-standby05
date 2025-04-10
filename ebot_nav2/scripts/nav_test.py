#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ebot_nav2.srv import Order 

class ButlerNode(Node):

    def __init__(self):
        super().__init__('butler_service_node')
        self.navigator = BasicNavigator()
        self.locations = {
            "kitchen": [1.43, 6.42, -1.53],
            "table1": [-0.58, -1.63, 0.0],
            "table2": [1.30, -5.46, 0.0],
            "table3": [-0.51, -5.28, 0.0],
            "home": [3.58, 1.57, 3.14]
        }

        self.server = self.create_service(Order, 'take_order', self.handle_order)
        self.get_logger().info(" Butler node ready for orders...")

        self.set_initial_pose()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info(" Navigation is active")


    def set_initial_pose(self):
        '''
        FUNCTION : it is to initiate nav2
        INPUT : NO INPUT
        OUTPUT : NO OUTPUT RETURNS
        '''

        home = self.locations["home"]
        pose = self.create_pose(home[0], home[1], home[2])
        self.navigator.setInitialPose(pose)

    def create_pose(self, x, y, theta):
        '''
        FUNCTION : it is to create pose for navigation
        INPUT : x,y,yaw
        OUTPUT : pose to navigate

        '''
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def go_to(self, location_name):
        '''
        FUNCTION : it is to navigate to the position or specific place
        INPUT : pose or position
        OUTPUT : result of navigation

        '''
        coords = self.locations[location_name]
        self.get_logger().info(f"➡️ Going to {location_name}...")
        goal_pose = self.create_pose(coords[0], coords[1], coords[2])
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f" Reached {location_name}")
            return True
        else:
            self.get_logger().warn(f" Failed to reach {location_name}")
            return False

    def wait_for_confirmation(self, timeout=15):
        '''
        FUNCTION : it is a function for verifiying the confirmation of the reciveing the order and deliverying the order
        INPUT : x,y,yaw
        OUTPUT : result of the order

        '''
        self.get_logger().info(f" Waiting for confirmation ({timeout}s)...")
        start = time.time()
        while time.time() - start < timeout:
            response = input("Type 'yes' to confirm or 'no' to reject: ").strip().lower()
            if response == "yes":
                return True
            elif response == "no":
                return False
        self.get_logger().warn(" Timeout waiting for confirmation.")
        return False

    def handle_order(self, request, response):
        '''
        FUNCTION : it is the callback function and processing will be done here
        INPUT : request and response of the service call
        OUTPUT : response of the request by the service call

        '''
        table_numbers = request.table_no
        self.get_logger().info(f" Received order for tables: {table_numbers}")

        for table in table_numbers:
            table_key = f"table{table}"
            if table_key not in self.locations:
                self.get_logger().warn(f" Invalid table: {table}")
                response.success = False
                response.message = f"Invalid table: {table}"
                return response

        if not self.go_to("kitchen"):
            response.success = False
            response.message = "Failed to go to kitchen"
            return response

        if not self.wait_for_confirmation():
            self.get_logger().info(" Kitchen rejected Returning home.")
            self.go_to("home")
            response.success = False
            response.message = "Kitchen rejected"
            return response
        
        for table in table_numbers:
            table_key = f"table{table}"
            if not self.go_to(table_key):
                response.success = False
                response.message = f" Failed to go to {table_key}"
                return response

            if not self.wait_for_confirmation():
                self.get_logger().info(f" Table {table_key} rejected Returning to kitchen then home.")
                self.go_to("kitchen")
                self.go_to("home")
                response.success = False
                response.message = f"{table_key} rejected"
                return response

            self.get_logger().info(f" Delivered to {table_key}")

        self.go_to("home")
        response.success = True
        response.message = " All deliveries completed successfully!"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ButlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
