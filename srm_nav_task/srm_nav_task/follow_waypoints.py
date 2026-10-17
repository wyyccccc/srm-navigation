import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class FollowWaypoints(Node):

    def __init__(self):
        super().__init__('follow_waypoints')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/red_standard_robot1/navigate_to_pose'
        )

        # =========================
        # 8字路线参数
        # =========================

        # 下柱中心
        lower_center = (1.0, 0.5)

        # 上柱中心
        upper_center = (1.0, 1.5)

        # 绕柱半径
        radius = 0.5

        # 每个圆多少个 waypoint
        points_per_circle = 8

        # 自动生成8字路线
        self.waypoints = self.generate_figure8_waypoints(
            lower_center,
            upper_center,
            radius,
            points_per_circle
        )

        self.current_index = 0

        self.get_logger().info('Waiting for Nav2 action server...')

        self._action_client.wait_for_server()

        self.get_logger().info(
            f'Generated {len(self.waypoints)} waypoints'
        )

        self.send_next_goal()

    def generate_figure8_waypoints(
        self,
        lower_center,
        upper_center,
        radius,
        points_per_circle
    ):

        waypoints = []

        lower_x, lower_y = lower_center
        upper_x, upper_y = upper_center

        # 两个柱子之间的中间点
        cross_x = (lower_x + upper_x) / 2.0
        cross_y = (lower_y + upper_y) / 2.0

        # =========================
        # 先到中间交叉点
        # =========================

        waypoints.append(
            (
                cross_x,
                cross_y,
                math.pi
            )
        )

        # =========================
        # 下柱：逆时针
        # =========================

        for i in range(points_per_circle):

            angle = (
                math.pi
                + 2.0 * math.pi * i / points_per_circle
            )

            x = lower_x + radius * math.cos(angle)
            y = lower_y + radius * math.sin(angle)

            # 车头沿圆的切线方向
            yaw = angle + math.pi / 2.0

            waypoints.append(
                (
                    x,
                    y,
                    yaw
                )
            )

        # 回到中间
        waypoints.append(
            (
                cross_x,
                cross_y,
                math.pi
            )
        )

        # =========================
        # 上柱：顺时针
        # =========================

        for i in range(points_per_circle):

            angle = (
                math.pi
                - 2.0 * math.pi * i / points_per_circle
            )

            x = upper_x + radius * math.cos(angle)
            y = upper_y + radius * math.sin(angle)

            # 顺时针切线方向
            yaw = angle - math.pi / 2.0

            waypoints.append(
                (
                    x,
                    y,
                    yaw
                )
            )

        # 最后回到中间
        waypoints.append(
            (
                cross_x,
                cross_y,
                math.pi
            )
        )

        return waypoints

    def send_next_goal(self):

        if self.current_index >= len(self.waypoints):

            self.get_logger().info(
                'All waypoints completed!'
            )

            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0

        goal_msg.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        goal_msg.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.get_logger().info(
            f'Sending waypoint '
            f'{self.current_index + 1}/'
            f'{len(self.waypoints)}: '
            f'x={x:.2f}, '
            f'y={y:.2f}, '
            f'yaw={yaw:.2f}'
        )

        self._send_goal_future = (
            self._action_client.send_goal_async(
                goal_msg
            )
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                f'Waypoint '
                f'{self.current_index + 1} '
                f'rejected'
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Waypoint '
            f'{self.current_index + 1} '
            f'accepted'
        )

        self._get_result_future = (
            goal_handle.get_result_async()
        )

        self._get_result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        # status == 4 表示成功
        if result.status == 4:

            self.get_logger().info(
                f'Waypoint '
                f'{self.current_index + 1} '
                f'reached'
            )

            self.current_index += 1

            self.send_next_goal()

        else:

            self.get_logger().error(
                f'Waypoint '
                f'{self.current_index + 1} '
                f'failed, '
                f'status={result.status}'
            )

            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = FollowWaypoints()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
