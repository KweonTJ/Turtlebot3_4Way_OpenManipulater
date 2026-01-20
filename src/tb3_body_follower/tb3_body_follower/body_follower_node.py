#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# TFLite 인터프리터 로드 (tflite_runtime 우선, 없으면 tensorflow 사용)
try:
    from tflite_runtime.interpreter import Interpreter
    TFLITE_RUNTIME = True
except ImportError:
    from tensorflow.lite import Interpreter
    TFLITE_RUNTIME = False


class Tb3BodyFollower(Node):
    def __init__(self):
        super().__init__('tb3_body_follower')

        # ======================
        # 파라미터
        # ======================
        self.declare_parameter('model_path', '/home/ubuntu/models/body_tracker.tflite')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('follow_min_duration', 3.0)   # 연속 인식 3초 → FOLLOW 진입
        self.declare_parameter('lost_timeout', 3.0)          # 미검출 3초 → IDLE 복귀
        self.declare_parameter('target_area', 0.12)          # 화면 내 사람 목표 비율
        self.declare_parameter('k_lin', 0.7)
        self.declare_parameter('k_ang', 1.3)
        self.declare_parameter('max_lin', 0.25)
        self.declare_parameter('max_ang', 1.8)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.follow_min_duration = self.get_parameter('follow_min_duration').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.target_area = self.get_parameter('target_area').value
        self.k_lin = self.get_parameter('k_lin').value
        self.k_ang = self.get_parameter('k_ang').value
        self.max_lin = self.get_parameter('max_lin').value
        self.max_ang = self.get_parameter('max_ang').value

        # ======================
        # TFLite 모델 로드
        # ======================
        self.get_logger().info(f'Loading TFLite model: {self.model_path}')
        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        in_shape = self.input_details[0]['shape']  # [1, H, W, C]
        in_dtype = self.input_details[0]['dtype']
        self.input_height = int(in_shape[1])
        self.input_width = int(in_shape[2])

        self.get_logger().info(f'Model input shape: {in_shape}, dtype: {in_dtype}')
        self.get_logger().info('Model output details:')
        for od in self.output_details:
            self.get_logger().info(str(od))

        # ======================
        # ROS 인터페이스
        # ======================
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # ======================
        # 상태 관리
        # ======================
        self.mode = 'IDLE'              # 'IDLE' or 'FOLLOW'
        self.last_detect_time = None    # 마지막으로 사람을 본 시각
        self.last_lost_time = None      # 마지막으로 사람을 못 본 시각

        self.get_logger().info('tb3_body_follower node started.')

    # -------------------------------------------------------------------------
    # TODO: body_tracker.tflite 출력에 맞게 구현해야 하는 부분
    # -------------------------------------------------------------------------
    def parse_body_tracker_output(self, outputs, frame_width, frame_height):
        """
        outputs: [output_tensor1, output_tensor2, ...] from TFLite
        frame_width, frame_height: 실제 카메라 이미지 크기

        return:
            detected: bool      # 사람 검출 여부
            cx_norm: float      # 0~1, 이미지 내 사람 중심 X (0: 왼쪽, 1: 오른쪽)
            cy_norm: float      # 0~1, 이미지 내 사람 중심 Y (0: 상단, 1: 하단)
            area_norm: float    # 0~1, 화면에서 사람이 차지하는 비율
        """

        # 현재는 placeholder. 실제 모델 출력 구조를 알게 되면 여기를 채워야 합니다.
        detected = False
        cx_norm = 0.5
        cy_norm = 0.5
        area_norm = 0.0

        # 예시 (모델이 normalized bbox를 준다고 가정할 때의 형태):
        # bboxes = outputs[0]  # shape: [1, N, 4]  (ymin, xmin, ymax, xmax)
        # scores = outputs[1]  # shape: [1, N]
        # classes = outputs[2] # shape: [1, N]
        # → person class만 골라서 가장 큰 bbox 선택 후 cx_norm, cy_norm, area_norm 계산

        return detected, cx_norm, cy_norm, area_norm

    # -------------------------------------------------------------------------

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w, _ = frame.shape

        # 전처리: uint8 기반
        input_tensor = self.preprocess(frame)

        # TFLite 실행
        self.interpreter.set_tensor(self.input_details[0]['index'], input_tensor)
        self.interpreter.invoke()

        outputs = []
        for od in self.output_details:
            out = self.interpreter.get_tensor(od['index'])
            outputs.append(out)

        # 출력 해석 → 사람 위치/크기 판단
        detected, cx_norm, cy_norm, area_norm = self.parse_body_tracker_output(
            outputs, w, h
        )

        now_sec = self.get_clock().now().seconds_nanoseconds()[0]

        # -----------------
        # 상태 전이 로직
        # -----------------
        if detected:
            if self.last_detect_time is None:
                self.last_detect_time = now_sec
            else:
                self.last_detect_time = now_sec

            # IDLE → FOLLOW 진입 조건
            if self.mode == 'IDLE':
                if self.last_lost_time is None:
                    self.last_lost_time = now_sec
                duration = now_sec - self.last_lost_time
                if duration >= self.follow_min_duration:
                    self.get_logger().info('Person detected continuously. Switch to FOLLOW mode.')
                    self.mode = 'FOLLOW'
        else:
            if self.last_lost_time is None:
                self.last_lost_time = now_sec
            else:
                self.last_lost_time = now_sec

            # FOLLOW → IDLE 복귀 조건
            if self.mode == 'FOLLOW':
                if self.last_detect_time is None:
                    self.last_detect_time = now_sec
                lost_duration = now_sec - self.last_detect_time
                if lost_duration >= self.lost_timeout:
                    self.get_logger().info('Person lost. Switch to IDLE mode.')
                    self.mode = 'IDLE'
                    self.publish_stop()

        # -----------------
        # FOLLOW 모드 제어
        # -----------------
        if self.mode == 'FOLLOW' and detected:
            self.follow_control(frame, cx_norm, cy_norm, area_norm)

    def preprocess(self, frame):
        """
        TFLite 모델이 요구하는 입력 형식에 맞게 전처리.
        현재 모델은 dtype=UINT8, shape=[1, 320, 320, 3]
        """
        # 리사이즈
        resized = cv2.resize(frame, (self.input_width, self.input_height))
        # BGR → RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # ★ float32/정규화 없이, uint8 그대로 사용
        input_data = rgb.astype(np.uint8)
        # 배치 차원 추가 [1, H, W, C]
        input_data = np.expand_dims(input_data, axis=0)
        return input_data

    def follow_control(self, frame, cx_norm, cy_norm, area_norm):
        """
        FOLLOW 모드에서 사람 중심 및 크기를 이용해 cmd_vel 생성.
        cx_norm: 0(왼쪽) ~ 1(오른쪽)
        cy_norm: 0(위) ~ 1(아래)  — 지금은 사용 안 해도 됨
        area_norm: 화면 내 사람 비율
        """
        # 좌우 오차
        error_x = (cx_norm - 0.5) / 0.5    # -1 ~ +1

        # 거리 오차 (area가 작으면 앞으로, 크면 뒤로)
        error_area = self.target_area - area_norm

        lin = self.k_lin * error_area
        ang = -self.k_ang * error_x

        lin = float(np.clip(lin, -self.max_lin, self.max_lin))
        ang = float(np.clip(ang, -self.max_ang, self.max_ang))

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_pub.publish(twist)

        # 필요하면 디버그 시각화 추가 가능
        # h, w, _ = frame.shape
        # cx = int(cx_norm * w)
        # cy = int(cy_norm * h)
        # cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        # cv2.imshow("Body Follower Debug", frame)
        # cv2.waitKey(1)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Tb3BodyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
