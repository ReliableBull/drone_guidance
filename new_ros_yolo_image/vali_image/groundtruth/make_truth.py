import cv2
import json
import os

# 마우스 콜백 함수
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, img, img_copy, coords, scale_width, scale_height

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            img = img_copy.copy()
            cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)
        coords.append({"x": int(ix ), "y": int(iy ), "width": int((x - ix)), "height":  int((y - iy) )})

# 이미지 파일 디렉토리 경로
dir_path = "./"


# 디렉토리 내의 모든 jpg 파일에 대해
for filename in os.listdir(dir_path):
    if filename.endswith('.jpg'):
        # 초기 변수 설정
        drawing = False
        ix, iy = -1, -1
        coords = []

        # 이미지 읽기
        img_path = os.path.join(dir_path, filename)
        img = cv2.imread(img_path)

        # 원본 이미지 크기 저장
        original_height, original_width = img.shape[:2]

        # # 이미지 크기 조정
        # new_width = 2000
        # new_height = int((new_width / original_width) * original_height)  # 원본 이미지의 비율 유지
        # img = cv2.resize(img, (new_width, new_height))

        # # 조정된 이미지 크기 저장
        # resize_height, resize_width = img.shape[:2]

        # # 원본 이미지와 조정된 이미지의 비율 계산
        # scale_width = original_width / resize_width
        # scale_height = original_height / resize_height

        # img_copy = img.copy()

        # 윈도우 생성 및 마우스 콜백 함수 연결
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', draw_rectangle)

        while(1):
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'키를 누르면 종료
                break

        cv2.destroyAllWindows()

        # JSON 파일로 저장
        json_filename = filename.rsplit('.', 1)[0] + '.json'
        json_path = os.path.join(dir_path, json_filename)
        with open(json_path, 'w') as f:
            json.dump(coords, f)
