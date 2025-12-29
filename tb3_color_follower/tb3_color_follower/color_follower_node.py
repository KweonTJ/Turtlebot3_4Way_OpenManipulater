#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np

# TFLite 라이브러리 임포트 (잘 작동했던 코드 방식 그대로)
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    from tensorflow.lite import Interpreter


class ColorFollowerNode(Node):
    def __init__(self):
        super().__init__('tb3_color_follower')

        # 1. 파라미터 선언
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('search_speed', 0.08)
        self.declare_parameter('k_ang', 0.0025)
        self.declare_parameter('k_lin', 0.0008)
        self.declare_parameter('lost_timeout', 1.0)
        self.declare_parameter('use_tflite', True)
        self.declare_parameter('color_model_path', '/home/ubuntu/models/color.tflite')

        # 2. 파라미터 값 가져오기
        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        self.search_speed = self.get_parameter('search_speed').value
        self.k_ang = self.get_parameter('k_ang').value
        self.k_lin = self.get_parameter('k_lin').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        
        self.use_tflite = self.get_parameter('use_tflite').value
        self.color_model_path = self.get_parameter('color_model_path').value

        self.state = 'SEARCH'
        self.last_blue_time = self.get_clock().now()
        self.last_bbox = None 
        self.bridge = CvBridge()

        # =========================================================
        # [핵심 수정] 잘 작동했던 코드(BodyFollower) 방식 그대로 설정
        # =========================================================
        if self.use_tflite:
            try:
                self.get_logger().info(f'Loading TFLite model: {self.color_model_path}')
                
                # 1. num_threads 옵션 완전 제거 (시스템 기본값 사용)
                self.interpreter = Interpreter(model_path=self.color_model_path)
                self.interpreter.allocate_tensors()
                
                self.input_details = self.interpreter.get_input_details()
                self.output_details = self.interpreter.get_output_details()
                
                # 2. 모델이 원하는 입력 크기와 타입 확인
                self.input_shape = self.input_details[0]['shape'] # [1, H, W, C]
                self.input_height = int(self.input_shape[1])
                self.input_width = int(self.input_shape[2])
                self.input_dtype = self.input_details[0]['dtype']
                
                self.get_logger().info(f'Model Input: {self.input_shape}, {self.input_dtype}')
                
            except Exception as e:
                self.get_logger().error(f'❌ 모델 로드 실패: {e}')
                self.use_tflite = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Subscriber (QoS: Best Effort 유지)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data 
        )

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f'🚀 노드 시작됨. 토픽 구독 중: {self.image_topic}')

    def image_callback(self, msg: Image):
        # [디버깅] 영상 수신 확인 (2초에 1번)
        self.get_logger().info("📷 영상 수신 중...", throttle_duration_sec=2.0)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 에러: {e}')
            return

        h, w, _ = cv_image.shape
        center_x = w / 2.0

        if self.use_tflite:
            bbox = self.detect_blue_with_tflite(cv_image)
        else:
            bbox = self.detect_blue_with_hsv(cv_image)

        if bbox is None:
            self.last_bbox = None
            return

        cx, bh, bw, area = bbox
        self.last_bbox = (cx, bh, bw, area, center_x)
        self.last_blue_time = self.get_clock().now()
        self.state = 'FOLLOW'

    def preprocess(self, frame):
        """
        잘 작동했던 코드의 전처리 로직 이식
        """
        # 1. 리사이즈
        resized = cv2.resize(frame, (self.input_width, self.input_height))
        
        # 2. BGR -> RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # 3. 데이터 타입 자동 맞춤 (핵심!)
        if self.input_dtype == np.uint8:
            # 모델이 uint8이면 그냥 씀 (나누기 255 안 함)
            input_data = rgb.astype(np.uint8)
        else:
            # 모델이 float32면 정규화 함
            input_data = rgb.astype(np.float32) / 255.0
            
        # 4. 차원 추가 [1, H, W, C]
        input_data = np.expand_dims(input_data, axis=0)
        return input_data

    def detect_blue_with_tflite(self, cv_image):
        if not hasattr(self, 'interpreter'): return None

        try:
            # 전처리 (함수로 분리하여 깔끔하게 적용)
            input_data = self.preprocess(cv_image)

            # 추론 실행
            # self.get_logger().info("🔥 추론 시작", throttle_duration_sec=1.0)
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            self.interpreter.invoke()
            
            # 결과 해석
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            
            # YOLO 출력 파싱
            # output shape: [N, 6] or [N, 85] depending on model
            # 보통 [x, y, w, h, conf, class_probs...]
            
            # 가장 신뢰도 높은 박스 찾기
            best_box_idx = np.argmax(output_data[:, 4])
            row = output_data[best_box_idx]
            
            obj_conf = row[4]
            class_scores = row[5:]
            class_id = np.argmax(class_scores)
            class_conf = class_scores[class_id]
            final_score = obj_conf * class_conf
            
            msg = f"👀 감지됨 | Class: {class_id} | Score: {final_score:.4f}"
            self.get_logger().info(msg, throttle_duration_sec=0.5)

            if final_score > 0.4:
                # 좌표 복원
                cx_pred, cy_pred, w_pred, h_pred = row[0], row[1], row[2], row[3]
                h_img, w_img, _ = cv_image.shape
                
                # 모델이 0~1 정규화 좌표를 주는지, 픽셀 좌표를 주는지 확인
                if cx_pred <= 1.0: 
                    cx = cx_pred * w_img
                    bw = w_pred * w_img
                    bh = h_pred * h_img
                else:
                    # 입력 이미지 크기(320 or 640) 기준 픽셀 좌표인 경우 비율로 변환
                    cx = (cx_pred / self.input_width) * w_img
                    bw = (w_pred / self.input_width) * w_img
                    bh = (h_pred / self.input_height) * h_img
                
                return (cx, bh, bw, bw*bh)

        except Exception as e:
            self.get_logger().error(f"❌ 추론 중 에러: {e}")
            return None
            
        return None

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()
        
        if self.state == 'SEARCH':
            cmd.linear.x = self.search_speed
            cmd.angular.z = 0.0
        elif self.state == 'FOLLOW':
            dt = (now - self.last_blue_time).nanoseconds * 1e-9
            if dt > self.lost_timeout:
                self.state = 'SEARCH'
                self.last_bbox = None
                self.get_logger().info('대상 놓침. SEARCH 모드 전환.', throttle_duration_sec=2.0)
                cmd.linear.x = self.search_speed
                cmd.angular.z = 0.0
            else:
                if self.last_bbox:
                    cx, bh, bw, area, center_x = self.last_bbox
                    # 제어 로직
                    err_x = cx - center_x
                    cmd.angular.z = float(-self.k_ang * err_x)
                    cmd.linear.x = float(self.k_lin * (160.0 - bh))
                    
                    cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.1)
                    cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()