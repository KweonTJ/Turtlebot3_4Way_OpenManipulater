import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

MODEL = "/home/ubuntu/models/color.tflite"

inter = tflite.Interpreter(model_path=MODEL)
inter.allocate_tensors()

input_details = inter.get_input_details()
output_details = inter.get_output_details()

input_index = input_details[0]['index']
output_index = output_details[0]['index']
ih, iw = input_details[0]['shape'][1:3]

cap = cv2.VideoCapture(0)  # front camera

print("=== Blue class ID 자동 탐지 시작 ===")
print("파란색 물체(옷/종이/큐브)를 카메라에 가까이 비춰주세요.")
print("클래스 점수를 분석합니다...\n")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    img = cv2.resize(frame, (iw, ih)).astype(np.float32)
    img = np.expand_dims(img, 0)

    inter.set_tensor(input_index, img)
    inter.invoke()

    output = inter.get_tensor(output_index)[0]   # (25200, 14)
    
    # objectness + class scores
    obj = output[:, 4]
    cls_scores = output[:, 5:]   # shape (25200, 9)

    # 후보 필터링 (objectness > 0.2)
    mask = obj > 0.2
    if np.sum(mask) < 10:
        continue

    avg_scores = np.mean(cls_scores[mask], axis=0)

    print("class 평균 점수:")
    for i, s in enumerate(avg_scores):
        print(f"  class {i}: {s:.4f}")
    print("==============================")

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()