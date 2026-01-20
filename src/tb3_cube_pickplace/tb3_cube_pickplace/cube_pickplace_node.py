#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

import tf2_ros
from rclpy.time import Time

# TODO: TFLite 모델 로딩용 (나중에)
import tflite_runtime.interpreter as tflite


class CubePickPlaceNode(Node):
    """
    EE 카메라에서 컬러 큐브(파랑/빨강/초록)를 탐지하면:
      1) TurtleBot 정지
      2) 큐브 2D 위치 -> EE 좌표계 3D 추정 (좌표 변환)
      3) 매니퓰레이터 Pick & Place 수행 (TODO: MoveIt 연동)
      4) 완료 후 다시 사람/색 추종 모드로 복귀 (연동 인터페이스 설계 필요)
    """

    def __init__(self):
        super().__init__('tb3_cube_pickplace')

        # 파라미터
        self.declare_parameter('ee_camera_topic', '/ee_camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('ee_camera_frame', 'ee_camera_link')
        self.declare_parameter('min_confidence', 0.7)

        self.declare_parameter('cube_model_path', '/home/ubuntu/models/cube.tflite')

        self.ee_camera_topic = self.get_parameter('ee_camera_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.ee_camera_frame = self.get_parameter('ee_camera_frame').value
        self.min_confidence = self.get_parameter('min_confidence').value

        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.ee_cam_sub = self.create_subscription(
            Image,
            self.ee_camera_topic,
            self.ee_camera_callback,
            10
        )

        # TFLite cube.tflite loading
        try:
            self.interpreter = tflite.Interpreter(model_path=self.cube_model_path)
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            self.get_logger().info(
                f'Loaded cube.tflite: input={self.input_details[0]["shape"]}, '
                f'dtype={self.input_details[0]["dtype"]}'
            )
            self.get_logger().info(f'Output details: {self.output_details}')
        except Exception as e:
            self.get_logger().error(f'Failed to load cube.tflite: {e}')
            self.interpreter = None

        self.get_logger().info('tb3_cube_pickplace node started.')

    def ee_camera_callback(self, msg: Image):
        # ROS Image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        if self.interpreter is None:
            # 모델이 없으면 일단 아무것도 안 함
            return

        # NEW: cube.tflite 로 큐브 후보 검출
        det = self.detect_cube_with_tflite(cv_image)
        if det is None:
            return

        cx, cy, w, h, cube_class, conf = det
        self.get_logger().info(
            f'Cube detected: class={cube_class}, conf={conf:.2f}, '
            f'pixel=({cx:.1f},{cy:.1f}), size=({w}x{h})'
        )

        # 1) TurtleBot 정지
        self.stop_robot()

        # 2) TODO: 좌표 변환 및 Pick & Place 호출
        #    - pixel (cx, cy) + depth/intrinsic + TF 를 이용해 base_link 기준 3D 좌표 추정
        #    - cube_class(파랑/빨강/초록)에 따라 다른 위치로 이동/배치

    def detect_cube_with_tflite(self, cv_image):
        """
        cube.tflite 을 실행해서 (cx, cy, w, h, class_id, confidence)를 반환.
        모델 구조를 모르기 때문에, 여기서는 입력 전처리 + 인터프리터 호출까지만 구현하고,
        출력 해석은 TODO로 남깁니다.
        """
        input_info = self.input_details[0]
        ih, iw = input_info['shape'][1], input_info['shape'][2]

        img_resized = cv2.resize(cv_image, (iw, ih))
        input_data = img_resized.astype(input_info['dtype'])

        if input_info['dtype'] == np.float32:
            input_data = input_data / 255.0

        input_data = np.expand_dims(input_data, axis=0)

        self.interpreter.set_tensor(input_info['index'], input_data)
        self.interpreter.invoke()

        output_info = self.output_details[0]
        output = self.interpreter.get_tensor(output_info['index'])

        # cube.tflite 의 출력 형태에 따라 아래 부분을 직접 구현해야 합니다.
        #
        # 예를 들어, YOLO 계열이면:
        #   output.shape == (1, N, 7)  정도일 수 있고,
        #   각 벡터가 [cx, cy, w, h, obj_score, cls0_score, cls1_score, ...] 일 수 있습니다.
        #
        # 이 경우:
        #   - obj_score * max(cls_scores) 가 min_confidence 이상인 박스만 후보로
        #   - 가장 큰 conf 를 가진 박스를 선택
        #   - cx, cy, w, h 를 원본 해상도로 환산
        #
        # TODO 예시 (가상의 형태):
        #
        # boxes = self.decode_yolo_like_output(output, cv_image.shape[1], cv_image.shape[0])
        # if not boxes:
        #     return None
        # best = boxes[0]  # (cx, cy, w, h, class_id, conf)
        # return best
        #
        self.get_logger().warn('detect_cube_with_tflite: output post-processing is not implemented yet.')
        return None

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Stop robot for pick & place.')

    # TODO: 실제 3D 좌표 계산 함수 (깊이/TF 필요)
    # def compute_cube_pose_in_base(self, u, v, depth):
    #     ...
    #     return pose_base


def main(args=None):
    rclpy.init(args=args)
    node = CubePickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
