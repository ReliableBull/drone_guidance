import json
import os

# JSON 파일이 있는 디렉토리 경로
dir_path = "./"

# 디렉토리 내의 모든 JSON 파일에 대해
for filename in os.listdir(dir_path):
    if filename.endswith('.json'):

        # JSON 파일 읽기
        json_path = os.path.join(dir_path, filename)
        with open(json_path, 'r') as f:
            data = json.load(f)

        # 새로운 텍스트 파일 작성
        txt_filename = filename.rsplit('.', 1)[0] + '.txt'
        txt_path = os.path.join(dir_path, txt_filename)
        with open(txt_path, 'w') as f:
            for obj in data:
                x = obj['x']
                y = obj['y']
                width = obj['width']
                height = obj['height']

                # width나 height 값이 음수인 경우에는 이를 양수로 변환하여 x와 y 좌표에 더함
                if width > 0 and height > 0:
                    f.write(f"Drone {x} {y} {x + width} {y + height}\n")                
                elif width < 0 and height > 0:
                    f.write(f"Drone {x + width} {y} {x} {y + height}\n")                
                elif width > 0 and height < 0:
                    f.write(f"Drone {x} {y + height} {x + width} {y}\n")                
                else:
                    f.write(f"Drone {x + width} {y + height} {x} {y} \n")                
