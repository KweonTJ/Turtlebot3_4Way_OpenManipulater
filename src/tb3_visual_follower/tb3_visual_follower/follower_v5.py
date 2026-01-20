# #!/usr/bin/env python3
# import traceback
# from typing import Optional, Tuple

# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge

# # TFLite 라이브러리 임포트
# try:
#     import tflite_runtime.interpreter as tflite
# except ImportError:
#     import tensorflow.lite as tflite


# class YoloV5Follower(Node):
#     def __init__(self):
#         super().__init__('yolov5_follower')

#         # === 모델 관련 설정 ===
#         self.model_path = '/home/ubuntu/models/color.tflite'
#         self.get_logger().info(f'Loading TFLite model: {self.model_path}')

#         self.interpreter = tflite.Interpreter(model_path=self.model_path)
#         self.interpreter.allocate_tensors()

#         self.input_details = self.interpreter.get_input_details()
#         self.output_details = self.interpreter.get_output_details()

#         # 일반적으로 input_details[0]['shape'] = [1, h, w, 3]
#         self.input_shape = self.input_details[0]['shape']
#         self.input_dtype = self.input_details[0]['dtype']
#         self.input_quant = self.input_details[0].get('quantization', (0.0, 0))  # (scale, zero_point)

#         self.get_logger().info(
#             f'Model input shape: {self.input_shape}, dtype: {self.input_dtype}, quant: {self.input_quant}'
#         )

#         # === ROS 설정 ===
#         # 카메라 이미지 구독 (실제 사용 중인 토픽으로 바꿔야 함)
#         # /image_raw 에서 프레임이 잘 나온다고 하셨으니 여기에 맞춤
#         self.sub_img = self.create_subscription(
#             Image, '/image_raw', self.img_cb, 10
#         )

#         # 속도 명령 발행
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

#         # YOLO 디버그 이미지 발행 (rqt_image_view에서 볼 토픽)
#         self.pub_debug = self.create_publisher(Image, '/yolo_debug', 10)

#         self.bridge = CvBridge()

#         # === 제어 파라미터 ===
#         self.target_width = 100.0
#         self.k_yaw = 0.018
#         self.k_dist = 0.015
#         self.max_spd = 0.25

#         # 디버그용 카운터
#         self.frame_count = 0

#         # 마지막으로 선택된 박스의 클래스/신뢰도 저장 (로그용)
#         self.last_cls_id = -1
#         self.last_conf = 0.0

#         self.get_logger().info('YOLOv5 Follower Started! Waiting for images...')

#     # ---------------------------------------------------------------------- #
#     # 이미지 콜백
#     # ---------------------------------------------------------------------- #
#     def img_cb(self, msg: Image):
#         self.frame_count += 1

#         # 1. ROS Image -> OpenCV 이미지로 변환
#         try:
#             frame = self._rosimg_to_cv(msg)
#         except Exception as e:
#             self.get_logger().error(f'Image convert error: {e}')
#             traceback.print_exc()
#             return

#         h, w, _ = frame.shape
#         center_x = w / 2.0

#         # 2. TFLite 입력 전처리
#         try:
#             input_tensor = self._preprocess(frame)
#         except Exception as e:
#             self.get_logger().error(f'Preprocess error: {e}')
#             traceback.print_exc()
#             return

#         # 3. 추론
#         try:
#             self.interpreter.set_tensor(self.input_details[0]['index'], input_tensor)
#             self.interpreter.invoke()
#             output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
#         except Exception as e:
#             self.get_logger().error(f'Inference error: {e}')
#             traceback.print_exc()
#             return

#         # 4. 후처리 (가장 큰 박스 하나 선택)
#         best_box = self._select_best_box(output_data, img_w=w, img_h=h)

#         cmd = Twist()

#         if best_box is not None:
#             x1, y1, x2, y2 = best_box
#             box_w = x2 - x1
#             box_cx = (x1 + x2) / 2.0

#             # 간단한 P제어
#             yaw_error = center_x - box_cx
#             dist_error = self.target_width - box_w

#             cmd.angular.z = yaw_error * self.k_yaw
#             cmd.linear.x = dist_error * self.k_dist

#             # 속도 제한
#             cmd.linear.x = max(min(cmd.linear.x, self.max_spd), -self.max_spd)
#             cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

#             # 디버그 박스 그리기
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

#             # 너무 자주 안 찍히도록 간단한 디버그 로그
#             if self.frame_count % 10 == 0:
#                 self.get_logger().info(
#                     f'DETECTED: cls_id={self.last_cls_id}, conf={self.last_conf:.2f}, '
#                     f'cx={box_cx:.1f}, w={box_w}, '
#                     f'cmd.linear.x={cmd.linear.x:.3f}, cmd.angular.z={cmd.angular.z:.3f}'
#                 )
#         else:
#             # 타겟이 없을 때는 정지
#             cmd.linear.x = 0.0
#             cmd.angular.z = 0.0
#             if self.frame_count % 30 == 0:
#                 self.get_logger().info('No detection. Stopping.')

#         # 5. 속도 명령 발행
#         self.pub_cmd.publish(cmd)

#         # 6. 디버그 이미지 발행
#         try:
#             debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#             self.pub_debug.publish(debug_msg)
#         except Exception as e:
#             self.get_logger().error(f'Debug image publish error: {e}')
#             traceback.print_exc()

#     # ---------------------------------------------------------------------- #
#     # ROS Image -> OpenCV BGR
#     # ---------------------------------------------------------------------- #
#     def _rosimg_to_cv(self, msg: Image) -> np.ndarray:
#         """
#         ROS Image 메시지를 OpenCV BGR 이미지로 변환.
#         YUYV(yuv422) 형식도 대응.
#         """
#         if msg.encoding in ('yuv422', 'yuv422_yuy2'):
#             # YUYV (YUV422) -> BGR 변환
#             frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
#             frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
#         else:
#             # 일반적인 경우
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         return frame

#     # ---------------------------------------------------------------------- #
#     # TFLite 입력 전처리
#     # ---------------------------------------------------------------------- #
#     def _preprocess(self, frame: np.ndarray) -> np.ndarray:
#         """
#         TFLite 모델 입력 형태로 전처리.
#         - 입력 dtype이 float32면: [0, 1] 범위로 스케일링
#         - 입력 dtype이 uint8이면: 0~255 uint8 그대로 사용
#         """
#         input_h = int(self.input_shape[1])
#         input_w = int(self.input_shape[2])

#         img = cv2.resize(frame, (input_w, input_h))
#         img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

#         if self.input_dtype == np.float32:
#             img = img.astype(np.float32) / 255.0
#         elif self.input_dtype == np.uint8:
#             img = img.astype(np.uint8)
#         else:
#             # 그 외 dtype은 여기에서 필요에 맞게 처리
#             self.get_logger().warn(f'Unexpected input dtype: {self.input_dtype}, casting to float32/255.0')
#             img = img.astype(np.float32) / 255.0

#         # 배치 차원 추가: (1, h, w, 3)
#         img = np.expand_dims(img, axis=0)
#         return img

#     # ---------------------------------------------------------------------- #
#     # 후처리: 가장 큰 박스 하나 선택
#     # ---------------------------------------------------------------------- #
#     def _select_best_box(
#         self, output_data: np.ndarray, img_w: int, img_h: int
#     ) -> Optional[Tuple[int, int, int, int]]:
#         """
#         TFLite 출력에서 가장 큰 박스를 하나 선택.
#         output_data: (N, 6 이상) 가정 [x1, y1, x2, y2, conf, cls, ...]
#         """
#         best_box = None
#         max_area = 0.0

#         if output_data is None or len(output_data) == 0:
#             return None

#         for det in output_data:
#             if len(det) < 6:
#                 continue

#             x1, y1, x2, y2, conf, cls_id = det[0], det[1], det[2], det[3], det[4], int(det[5])

#             # 신뢰도 필터
#             if conf < 0.3:
#                 continue

#             # body_tracker.tflite 이면 보통 0이 person일 가능성이 큼
#             # 사람만 추종하고 싶으면 아래 주석 해제:
#             # if cls_id != 0:
#             #     continue

#             # 좌표 스케일링
#             if x1 <= 1.0 and x2 <= 1.0 and y1 <= 1.0 and y2 <= 1.0:
#                 x1 = int(x1 * img_w)
#                 x2 = int(x2 * img_w)
#                 y1 = int(y1 * img_h)
#                 y2 = int(y2 * img_h)
#             else:
#                 x1 = int(x1)
#                 x2 = int(x2)
#                 y1 = int(y1)
#                 y2 = int(y2)

#             # 잘못된 박스 필터링
#             if x2 <= x1 or y2 <= y1:
#                 continue

#             area = (x2 - x1) * (y2 - y1)
#             if area > max_area:
#                 max_area = area
#                 best_box = (x1, y1, x2, y2)

#                 # ★ 로그용으로 마지막 cls/conf 저장
#                 self.last_cls_id = cls_id
#                 self.last_conf = float(conf)

#         return best_box


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloV5Follower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import traceback
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
# [필수] QoS 설정을 세밀하게 하기 위해 임포트
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite


class RealTimeYoloFollower(Node):
    def __init__(self):
        super().__init__('yolov5_follower')

        cv2.setNumThreads(0)

        # === 모델 설정 ===
        self.model_path = '/home/ubuntu/models/color.tflite'
        self.get_logger().info(f'Loading Model: {self.model_path}')

        self.interpreter = tflite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_shape = self.input_details[0]['shape']
        self.input_dtype = self.input_details[0]['dtype']

        # === [핵심 수정 1] QoS 설정을 Depth=1로 강제 (실시간성 확보) ===
        # 이전 데이터는 과감히 버리고, 무조건 '최신 데이터' 1개만 받습니다.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_img = self.create_subscription(
            Image, '/image_raw', self.img_cb, qos_profile
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # === 제어 파라미터 ===
        self.target_width = 100.0
        self.k_yaw = 0.015
        self.k_dist = 0.012
        self.max_spd = 0.22
        
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.last_detection_time = self.get_clock().now()

        # 제어 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('🚀 실시간 팔로워 (Depth=1, 좌표보정) 시작!')

    def img_cb(self, msg: Image):
        try:
            # 1. 변환
            if msg.encoding in ('yuv422', 'yuv422_yuy2'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            h_img, w_img, _ = frame.shape
            center_x = w_img / 2.0

            # 2. 전처리
            input_h = int(self.input_shape[1])
            input_w = int(self.input_shape[2])
            
            img_resized = cv2.resize(frame, (input_w, input_h), interpolation=cv2.INTER_NEAREST)
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)

            if self.input_dtype == np.float32:
                input_data = img_rgb.astype(np.float32) / 255.0
            else:
                input_data = img_rgb.astype(np.uint8)

            input_tensor = np.expand_dims(input_data, axis=0)
            input_tensor = np.ascontiguousarray(input_tensor)

            # 3. 추론
            self.interpreter.set_tensor(self.input_details[0]['index'], input_tensor)
            self.interpreter.invoke()
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

            # 4. 후처리
            best_box = None
            max_area = 0.0

            for det in output_data:
                if len(det) < 6: continue
                
                cx, cy, w, h, obj_conf = det[0], det[1], det[2], det[3], det[4]
                class_scores = det[5:]
                cls_id = np.argmax(class_scores)
                conf = obj_conf * class_scores[cls_id]
                
                if conf < 0.4: continue 

                # === [핵심 수정 2] 좌표계 자동 보정 ===
                # 만약 모델이 0~1이 아니라 픽셀좌표(예: 320.0)를 뱉고 있다면 정규화해줍니다.
                if cx > 1.0 or cy > 1.0 or w > 1.0 or h > 1.0:
                    cx = cx / input_w
                    cy = cy / input_h
                    w = w / input_w
                    h = h / input_h

                # 좌표 복원 (정규화된 값을 실제 이미지 크기로 변환)
                # YOLO는 중심점(cx) 기준이므로 좌상단/우하단으로 변환해야 함
                x1 = int((cx - w/2) * w_img)
                x2 = int((cx + w/2) * w_img)
                y1 = int((cy - h/2) * h_img)
                y2 = int((cy + h/2) * h_img)

                # 박스 유효성 검사
                if x2 <= x1 or y2 <= y1: continue

                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    best_box = (x1, x2) 

            # 5. 목표값 업데이트
            if best_box is not None:
                x1, x2 = best_box
                box_w = x2 - x1
                box_cx = (x1 + x2) / 2.0

                self.last_detection_time = self.get_clock().now()
                
                # 목표값 설정
                self.target_angular = (center_x - box_cx) * self.k_yaw
                self.target_linear = (self.target_width - box_w) * self.k_dist
                
                # 최대 속도 제한
                self.target_linear = max(min(self.target_linear, self.max_spd), -self.max_spd)
                self.target_angular = max(min(self.target_angular, 1.2), -1.2)

        except Exception:
            pass 

    def control_loop(self):
        """
        비동기 제어 루프 (20Hz)
        """
        now = self.get_clock().now()
        dt_duration = (now - self.last_detection_time).nanoseconds / 1e9

        # 0.5초 이상 놓치면 정지
        if dt_duration > 0.5:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # Low Pass Filter (부드러운 움직임)
        alpha = 0.4 
        self.current_linear = (alpha * self.target_linear) + ((1 - alpha) * self.current_linear)
        self.current_angular = (alpha * self.target_angular) + ((1 - alpha) * self.current_angular)

        if abs(self.current_linear) < 0.01: self.current_linear = 0.0
        if abs(self.current_angular) < 0.01: self.current_angular = 0.0

        cmd = Twist()
        cmd.linear.x = float(self.current_linear)
        cmd.angular.z = float(self.current_angular)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeYoloFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()