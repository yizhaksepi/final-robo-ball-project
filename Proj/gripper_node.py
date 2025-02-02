#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import robotiq_gripper
from robotiq_hande_ros_driver.srv import GripperService  # Adjusted service import for ROS 2

class HandEGripper(Node):
    def __init__(self):
        super().__init__('hand_e_gripper_node')
        # get the IP
        self.declare_parameter('robot_ip', '192.168.0.1')  # Default IP, change as necessary
        ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # initialize the gripper
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.get_logger().info("Connecting to the gripper.....")
        self.gripper.connect(ip, 63352)
        self.get_logger().info("Activating the gripper.....")
        self.gripper.activate(auto_calibrate=False)
        
        # set up server
        self.gripper_server = self.create_service(GripperService, 'gripper_service', self.server_callback)
        self.get_logger().info("Gripper ready to receive service request...")
    
    def server_callback(self, request, response):
        pos = request.position
        speed = request.speed
        force = request.force
        if speed > 255 or speed <= 0:
            response.message = 'Invalid speed value. Valid in range (0, 255]'
            return response
        if force > 255 or force <= 0:
            response.message = 'Invalid force value. Valid in range (0, 255]'
            return response
        if pos > 255 or pos < 0:
            response.message = 'Invalid position value. Valid in range [0, 255]'
            return response

        self.get_logger().info(f"Moving the gripper. position = {pos}, speed = {speed}, force = {force}")
        self.gripper.move_and_wait_for_pos(pos, speed, force)
        response.message = 'Done'
        return response

def main(args=None):
    rclpy.init(args=args)
    gripper_obj = HandEGripper()
    try:
        rclpy.spin(gripper_obj)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

