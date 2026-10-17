import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import math


class SendGoal(Node):

    def __init__(self):
        super().__init__('send_goal')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/red_standard_robot1/navigate_to_pose'
        )

        self.send_goal()

    def send_goal(self):

        self.get_logger().info('Waiting for Nav2 action server...')

        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 目标位置
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.position.z = 0.0

        # 目标朝向 yaw = 0°
        yaw = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Sending goal: x=1.0, y=0.0')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info('Goal finished')

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = SendGoal()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
