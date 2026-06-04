import cv2
import numpy as np


class LineTracker:
    def __init__(self, w=320, h=240):
        self.w = w
        self.h = h
        self.kernel_matrix = np.ones((5, 5), np.uint8)

    def process(self, frame, debug=True):
        frame = cv2.resize(frame, (self.w, self.h))
        display_frame = frame.copy() if debug else None

        roi_top = int(self.h * 0.7)
        roi = frame[roi_top:self.h, 0:self.w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary_frame = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        cleaned_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_OPEN, self.kernel_matrix)

        contours, _ = cv2.findContours(cleaned_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if debug:
            cv2.rectangle(display_frame, (0, roi_top), (self.w, self.h), (255, 0, 0), 2)
            cv2.line(display_frame, (self.w // 2, 0), (self.w // 2, self.h), (0, 0, 255), 1)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    error = cx - (self.w // 2)

                    if debug:
                        actual_y = roi_top + int(self.h * 0.15)
                        cv2.circle(display_frame, (cx, actual_y), 6, (0, 255, 0), -1)
                        cv2.line(display_frame, (self.w // 2, actual_y), (cx, actual_y), (0, 255, 255), 2)

                    return error, True, display_frame

        return 0, False, display_frame