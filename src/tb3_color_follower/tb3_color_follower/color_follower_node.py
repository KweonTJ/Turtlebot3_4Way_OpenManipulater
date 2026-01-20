#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVColorFollower(Node):
    def __init__(self):
        super().__init__('tb3_hsv_follower')

        # === 파라미터 ===
        # Hue(색상): 110~130 (파랑)
        self.lower_blue = np.array([110, 40, 70])  
        self.upper_blue = np.array([135, 255, 255])

        self.k_ang = 0.003
        self.max_spd = 0.15
        self.min_area = 2000 

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.accel_factor = 0.1

        self.sub_img = self.create_subscription(
            Image, '/image_raw', self.img_cb, qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # [수정] 디버그 화면을 '마스크'가 아닌 '결과 영상'으로 변경
        self.pub_debug = self.create_publisher(Image, '/camera/debug_result', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('🚀 HSV Color Follower (Box Display) 시작!')

    def img_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        h, w, _ = cv_image.shape
        image_center_x = w / 2

        # 전처리
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_linear = 0.0
        target_angular = 0.0
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > self.min_area:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0

                # === [추가됨] 바운딩 박스 그리기 ===
                x, y, bw, bh = cv2.boundingRect(c)
                # 초록색 박스 (Green), 두께 2
                cv2.rectangle(cv_image, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                
                # 중심점 (빨간점)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                
                # 텍스트 정보 (면적)
                label = f"Blue Area: {int(area)}"
                cv2.putText(cv_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 제어 로직
                err_x = image_center_x - cx
                target_angular = float(err_x * self.k_ang)

                if area < 60000: 
                    target_linear = 0.15
                else:
                    target_linear = 0.0

        # 속도 스무딩 (Low Pass Filter)
        self.current_linear = (self.current_linear * (1 - self.accel_factor)) + (target_linear * self.accel_factor)
        self.current_angular = (self.current_angular * (1 - self.accel_factor)) + (target_angular * self.accel_factor)

        if abs(self.current_linear) < 0.001: self.current_linear = 0.0
        if abs(self.current_angular) < 0.001: self.current_angular = 0.0

        self.current_linear = min(self.current_linear, self.max_spd)
        self.current_angular = max(min(self.current_angular, 1.5), -1.5)

        cmd = Twist()
        cmd.linear.x = float(self.current_linear)
        cmd.angular.z = float(self.current_angular)
        self.pub_cmd.publish(cmd)

        # [수정] 박스가 그려진 컬러 이미지를 발행 (rqt에서 확인용)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.pub_debug.publish(debug_msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HSVColorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()