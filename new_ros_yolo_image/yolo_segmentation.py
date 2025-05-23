from ultralytics import YOLO
import numpy as np


class YOLOSegmentation:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def detect(self, img):
        # Get img shape
        height, width, channels = img.shape
        results = self.model.predict(source=img.copy(), save=False, save_txt=False)
        result = results[0]
        # print(len(result))
        segmentation_contours_idx = []

        if len(result) == 0:
            return -1, -1, -1, -1 , 0

        for seg in result.masks.segments:
            # contours
            seg[:, 0] *= width
            seg[:, 1] *= height
            segment = np.array(seg, dtype=np.int32)
            segmentation_contours_idx.append(segment)

        bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
        # Get class ids
        class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
        # Get scores
        scores = np.array(result.boxes.conf.cpu(), dtype="float").round(2)
        return bboxes, class_ids, segmentation_contours_idx, scores, 1
