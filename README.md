# 개요
- 일상 및 작업 환경에서 유연하게 도움을 주는 이동 조작 통합 로봇
- HSV를 활용하여 사람을 따라 이동 및 탐지된 물체에 대해 Pick & Place 
- 4WD 커스텀 플랫폼을 적용하여 더욱 강력한 힘 제공

***
# 환경 설정
## Software
- ROS2 Humble
- Python 3.10
- TF Lite Model
- Yolo v5
## Hardware
- Turtlebot3 Burger 4WD
- Open Manipulator X
- LiDAR
- RGB Camera
- Raspberry Pi 4B 8GB
***
# 활용 알고리즘
- HSV Algorithm
	- 특정 색상 추출 및 객체 추적 영상 처리 기술 활용
	- 조명 변화에 강해 특정 색상을 안정적으로 인식하고 분리하는데 유용
	- 영상에서 특정 색상을 가진 객체를 지속적으로 추적하여 Follower하여 따라감
	- 원하는 색상의 HSV 범위 설정, 해당 범위에 해당하는 픽셀 추출 마스크를 생성하여 배경과 특정 색상의 물체를 분리

***
# 패키지 생성
## tb3_color_follower
- HSV 를 활용한 Color follower 를 활용하여 특정 색상을 가진 사람을 따라간다. 
- 전면에 달린 야간 카메라를 활용하여, 상대적으로 어두운 환경에서도 color follower를 정확하게 추정 및 추적


