#!/usr/bin/env python3

"""
Control Service Node for Humanoid Robot

This node provides services for controlling the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist


class ControlService(Node):
    def __init__(self):
        super().__init__('control_service')

        # Create services
        self.enable_service = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback
        )

        self.reset_service = self.create_service(
            Trigger,
            'reset_robot',
            self.reset_robot_callback
        )

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_enabled = False
        self.get_logger().info('Control Service Node has been started')

    def enable_robot_callback(self, request, response):
        """Callback to enable/disable the robot"""
        self.robot_enabled = request.data
        response.success = True
        if self.robot_enabled:
            response.message = 'Robot enabled successfully'
            self.get_logger().info('Robot enabled')
        else:
            response.message = 'Robot disabled successfully'
            self.get_logger().info('Robot disabled')
        return response

    def reset_robot_callback(self, request, response):
        """Callback to reset the robot to initial position"""
        if not self.robot_enabled:
            response.success = False
            response.message = 'Cannot reset: Robot is disabled'
            return response

        # Reset joint positions to initial state (this would typically involve
        # commanding the joints to their home positions)
        self.get_logger().info('Resetting robot to initial position')

        # Publish zero velocity to stop any movement
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

        response.success = True
        response.message = 'Robot reset successfully'
        return response


def main(args=None):
    rclpy.init(args=args)

    control_service = ControlService()

    try:
        rclpy.spin(control_service)
    except KeyboardInterrupt:
        pass
    finally:
        control_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()