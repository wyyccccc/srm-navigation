import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


class Figure8Navigation(Node):

    def __init__(self):
        super().__init__('figure8_navigation')

        self.client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


class Figure8Navigation(Node):

    def __init__(self):
        super().__init__('figure8_navigation')

        self.client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )

        self.get_logger().info(
            'Waiting for /follow_waypoints...'
        )

        self.client.wait_for_server()

        self.get_logger().info(
            'Nav2 ready.'
        )

        self.send_figure8()

    def make_pose(self, x, y):

        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose

    def add_arc(
        self,
        points,
        cx,
        cy,
        ux,
        uy,
        radius,
        start_angle,
        end_angle,
        num_points,
        skip_first=False
    ):

        # 与两根桩连线垂直的单位向量
        nx = -uy
        ny = ux

        start_index = 1 if skip_first else 0

        for i in range(
            start_index,
            num_points
        ):

            ratio = (
                i / (num_points - 1)
            )

            angle = (
                start_angle
                + ratio
                * (
                    end_angle
                    - start_angle
                )
            )

            x = (
                cx
                + radius
                * math.cos(angle)
                * ux
                + radius
                * math.sin(angle)
                * nx
            )

            y = (
                cy
                + radius
                * math.cos(angle)
                * uy
                + radius
                * math.sin(angle)
                * ny
            )

            points.append(
                (x, y)
            )

    def send_figure8(self):

        # ==============================================
        # 现场测得的数据
        # ==============================================

        # 停车位
        parking_x = 0.896
        parking_y = 0.108

        # 第一根桩：先绕
        pole1_x = 0.461
        pole1_y = 0.285

        # 第二根桩：后绕
        pole2_x = 1.321
        pole2_y = 0.301

        # ==============================================
        # 自动计算两桩方向
        # ==============================================

        dx = (
            pole2_x
            - pole1_x
        )

        dy = (
            pole2_y
            - pole1_y
        )

        distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        if distance < 0.1:

            self.get_logger().error(
                'Pole positions are invalid!'
            )

            rclpy.shutdown()
            return

        # pole1 -> pole2 的单位方向向量
        ux = dx / distance
        uy = dy / distance

        # 八字交叉点 = 两桩中点
        cross_x = (
            pole1_x
            + pole2_x
        ) / 2.0

        cross_y = (
            pole1_y
            + pole2_y
        ) / 2.0

        # 让两个环经过交叉点
        radius = distance / 2.0

        self.get_logger().info(
            f'Pole distance = '
            f'{distance:.3f} m'
        )

        self.get_logger().info(
            f'Radius = '
            f'{radius:.3f} m'
        )

        self.get_logger().info(
            f'Cross point = '
            f'({cross_x:.3f}, '
            f'{cross_y:.3f})'
        )

        points = []

        # ==============================================
        # 1. 八字入口
        # ==============================================

        # 第一根桩远离第二根桩的一侧
        entry_x = (
            pole1_x
            - radius * ux
        )

        entry_y = (
            pole1_y
            - radius * uy
        )

        # 从停车位导航到入口点
        points.append(
            (entry_x, entry_y)
        )

        # ==============================================
        # 2. 第一根桩：第一半圈
        #
        # entry -> cross
        #
        # 逆时针
        # ==============================================

        self.add_arc(
            points,
            pole1_x,
            pole1_y,
            ux,
            uy,
            radius,
            math.pi,
            2.0 * math.pi,
            6,
            skip_first=True
        )

        # 此时最后一个点已经是 cross
        # 不再额外 append cross

        # ==============================================
        # 3. 第二根桩：完整一圈
        #
        # cross -> cross
        #
        # 顺时针
        # ==============================================

        self.add_arc(
            points,
            pole2_x,
            pole2_y,
            ux,
            uy,
            radius,
            math.pi,
            -math.pi,
            10,
            skip_first=True
        )

        # 此时最后一点已经回到 cross
        # 不再重复添加

        # ==============================================
        # 4. 第一根桩：剩余半圈
        #
        # cross -> entry
        #
        # 继续逆时针
        # ==============================================

        self.add_arc(
            points,
            pole1_x,
            pole1_y,
            ux,
            uy,
            radius,
            0.0,
            math.pi,
            6,
            skip_first=True
        )

        # 此时最后一点已经是 entry
        # 不再重复添加

        # ==============================================
        # 5. 回停车位
        # ==============================================

        points.append(
            (
                parking_x,
                parking_y
            )
        )

        # ==============================================
        # 输出 waypoint
        # ==============================================

        self.get_logger().info(
            f'Total waypoints: '
            f'{len(points)}'
        )

        for i, (x, y) in enumerate(
            points
        ):

            self.get_logger().info(
                f'P{i + 1}: '
                f'({x:.3f}, '
                f'{y:.3f})'
            )

        # ==============================================
        # FollowWaypoints
        # ==============================================

        goal = (
            FollowWaypoints.Goal()
        )

        for x, y in points:

            goal.poses.append(
                self.make_pose(
                    x,
                    y
                )
            )

        self.get_logger().info(
            'Sending competition '
            'figure-8...'
        )

        future = (
            self.client.send_goal_async(
                goal,
                feedback_callback=
                    self.feedback_callback
            )
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(
        self,
        feedback_msg
    ):

        current = (
            feedback_msg
            .feedback
            .current_waypoint
        )

        self.get_logger().info(
            f'Current waypoint: '
            f'{current}'
        )

    def goal_response_callback(
        self,
        future
    ):

        goal_handle = (
            future.result()
        )

        if not goal_handle.accepted:

            self.get_logger().error(
                'Goal rejected.'
            )

            return

        self.get_logger().info(
            'Goal accepted.'
        )

        result_future = (
            goal_handle
            .get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(
        self,
        future
    ):

        wrapped_result = (
            future.result()
        )

        result = (
            wrapped_result.result
        )

        status = (
            wrapped_result.status
        )

        if (
            len(
                result.missed_waypoints
            )
            == 0
        ):

            self.get_logger().info(
                'COMPETITION FIGURE-8 '
                'FINISHED!'
            )

        else:

            self.get_logger().warn(
                f'Missed waypoints: '
                f'{result.missed_waypoints}'
            )

        self.get_logger().info(
            f'Action status: {status}'
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        Figure8Navigation()
    )

    rclpy.spin(
        node
    )


if __name__ == '__main__':
    main()
