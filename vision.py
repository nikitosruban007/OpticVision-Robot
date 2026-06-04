import cv2
import numpy as np


class LineTracker:
    def __init__(self, w=320, h=240):
        self.w = w
        self.h = h
        self.kernel_matrix = np.ones((5, 5), np.uint8)

    def process(self, frame):
        frame = cv2.resize(frame, (self.w, self.h))
        roi = frame[int(self.h * 0.7):self.h, 0:self.w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary_frame = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        cleaned_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_OPEN, self.kernel_matrix)

        contours, _ = cv2.findContours(cleaned_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    error = cx - (self.w // 2)
                    return error, True, cleaned_frame

        return 0, False, cleaned_frame