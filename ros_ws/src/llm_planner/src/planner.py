#!/usr/bin/env python3

"""
LLM-based Cognitive Planner Node

This node implements a conceptual LLM-based cognitive planner that takes
voice commands and generates task plans for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import json
import re


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Create subscriber for voice transcriptions
        self.voice_sub = self.create_subscription(
            String,
            '/voice/transcription',
            self.voice_callback,
            10
        )

        # Create publisher for navigation goals
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Create publisher for task plans
        self.task_plan_pub = self.create_publisher(
            String,
            '/task_plan',
            10
        )

        # Command mapping for demo purposes
        self.command_mapping = {
            'move forward': self.handle_move_forward,
            'turn left': self.handle_turn_left,
            'turn right': self.handle_turn_right,
            'stop': self.handle_stop,
            'go to the kitchen': self.handle_goto_kitchen,
            'pick up the red object': self.handle_pickup_red_object
        }

        self.get_logger().info('LLM Planner Node initialized')

    def voice_callback(self, msg):
        """Callback for voice transcription"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: "{command}"')

        # Find the best matching command handler
        handler = None
        for cmd, func in self.command_mapping.items():
            if cmd in command:
                handler = func
                break

        if handler:
            self.get_logger().info(f'Executing command handler for: {command}')
            handler(command)
        else:
            self.get_logger().warning(f'Unknown command: {command}')
            self.handle_unknown_command(command)

    def handle_move_forward(self, command):
        """Handle move forward command"""
        # Create a simple navigation goal (1 meter forward)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.0  # Move 1 meter forward
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.nav_goal_pub.publish(goal)
        self.publish_task_plan(f'Navigating forward by 1 meter')

    def handle_turn_left(self, command):
        """Handle turn left command"""
        # Create a navigation goal to turn left
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        # Simple rotation (this would need proper quaternion math in practice)
        goal.pose.orientation.z = 0.707  # 90 degrees left
        goal.pose.orientation.w = 0.707

        self.nav_goal_pub.publish(goal)
        self.publish_task_plan(f'Turning left')

    def handle_turn_right(self, command):
        """Handle turn right command"""
        # Create a navigation goal to turn right
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        # Simple rotation (this would need proper quaternion math in practice)
        goal.pose.orientation.z = -0.707  # 90 degrees right
        goal.pose.orientation.w = 0.707

        self.nav_goal_pub.publish(goal)
        self.publish_task_plan(f'Turning right')

    def handle_stop(self, command):
        """Handle stop command"""
        # Publish an empty task plan to indicate stopping
        self.publish_task_plan(f'Stopping current action')

    def handle_goto_kitchen(self, command):
        """Handle go to kitchen command"""
        # Create a navigation goal to go to the kitchen
        # (coordinates would be defined based on the map)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 5.0  # Example kitchen coordinates
        goal.pose.position.y = 3.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.nav_goal_pub.publish(goal)
        self.publish_task_plan(f'Navigating to kitchen at (5.0, 3.0)')

    def handle_pickup_red_object(self, command):
        """Handle pick up red object command"""
        # This would involve navigation, object detection, and manipulation
        # For now, just publish a task plan
        self.publish_task_plan(f'Searching for red object, navigating to it, and attempting pickup')

    def handle_unknown_command(self, command):
        """Handle unknown command"""
        self.publish_task_plan(f'Unknown command: {command}. Available commands: move forward, turn left, turn right, stop, go to the kitchen, pick up the red object')

    def publish_task_plan(self, plan_description):
        """Publish a task plan"""
        plan_msg = String()
        plan_msg.data = json.dumps({
            'timestamp': self.get_clock().now().nanoseconds,
            'plan': plan_description,
            'status': 'active'
        })
        self.task_plan_pub.publish(plan_msg)
        self.get_logger().info(f'Published task plan: {plan_description}')


def main(args=None):
    rclpy.init(args=args)

    planner_node = LLMPlannerNode()

    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()