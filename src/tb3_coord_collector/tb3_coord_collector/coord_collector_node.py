#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class CoordCollectorNode(Node):
    def __init__(self):
        super().__init__('tb3_coord_collector')

        # ★ 기본 world_frame = base_link (tf2_echo에서 잘 되는 조합 그대로)
        self.world_frame = self.declare_parameter(
            'world_frame',
            'base_link'
        ).value

        # ★ EE 프레임 이름 그대로
        self.target_frames = self.declare_parameter(
            'target_frames',
            ['end_effector_link']
        ).value

        # TF 버퍼 & 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF 프레임 목록을 한 번만 찍어보는 타이머 (디버깅용)
        self.printed_frames = False
        self.frames_timer = self.create_timer(2.0, self.print_frames_once)

        # 프레임별 PoseStamped 퍼블리셔
        self.pose_publishers = {}
        for frame in self.target_frames:
            topic_name = f'/tb3_coord/{frame}'
            self.pose_publishers[frame] = self.create_publisher(
                PoseStamped,
                topic_name,
                10
            )
            self.get_logger().info(
                f'Publishing pose of "{frame}" in "{self.world_frame}" to "{topic_name}"'
            )

        # 10Hz 타이머
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def print_frames_once(self):
        """TF 버퍼 안에 어떤 프레임들이 있는지 한 번만 출력."""
        if self.printed_frames:
            return
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f"TF frames in buffer:\n{frames_yaml}")
            self.printed_frames = True
        except Exception as e:
            self.get_logger().warn(f"Could not get TF frames yet: {e}")

    def timer_callback(self):
        now = self.get_clock().now()

        for frame in self.target_frames:
            pose = self.lookup_pose(self.world_frame, frame, now)

            if pose is None:
                continue

            self.pose_publishers[frame].publish(pose)
            self.log_pose(frame, pose)

    def lookup_pose(self, world_frame: str, target_frame: str, now: Time):
        """
        world_frame -> target_frame 변환을 TF에서 읽어서 PoseStamped로 변환
        """
        try:
            # latest TF 사용 (시간 0)
            transform = self.tf_buffer.lookup_transform(
                world_frame,       # target_frame
                target_frame,      # source_frame
                rclpy.time.Time()  # time=0 → 최신
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform from "{world_frame}" to "{target_frame}": {ex}'
            )
            return None

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = transform.header.frame_id  # world_frame

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z

        pose.pose.orientation = transform.transform.rotation

        return pose

    def log_pose(self, name: str, pose: PoseStamped):
        self.get_logger().info(
            f'{name} pose in {self.world_frame}: '
            f'pos=({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, '
            f'{pose.pose.position.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 여기서 shutdown 두 번 안 부르도록 그냥 한 번만 호출
        rclpy.shutdown()
