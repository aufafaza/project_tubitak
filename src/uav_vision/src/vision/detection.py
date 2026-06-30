import cv2
import numpy as np


class Detect: 
    videoCapture = None;
    rectangle = False;
    
    def __init__(self, videoSource, rectangle: bool): 
        self.videoCapture = videoSource;  
        self.rectangle = rectangle; 

    def maskRed(self, bgr): 
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); 
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15));
        maskLowerBound1 = np.array([0, 100, 80])
        maskUpperBound1 = np.array([15, 255, 255])
        maskLowerBound2 = np.array([165, 100, 80])
        maskUpperBound2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, maskLowerBound1, maskUpperBound1) 
        mask2 = cv2.inRange(hsv, maskLowerBound2, maskUpperBound2) 
        mask =  cv2.bitwise_or(mask1, mask2)
        k = np.ones((3, 3), np.uint8)
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, mask)
        cv2.erode(mask, kernel, mask); 
        cv2.dilate(mask, kernel, mask); 

        return mask

    def maskBlue(self, bgr): 
        # blur = cv2.GaussianBlur(bgr, (7, 7), 0)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV) 
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        maskLowerBound = np.array([100, 120, 50])
        maskUpperBound = np.array([130, 255, 255])
        
        mask = cv2.inRange(hsv, maskLowerBound, maskUpperBound)
        k = np.ones((3, 3), np.uint8)
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, mask) 
        cv2.erode(mask, kernel, mask) 
        cv2.dilate(mask, kernel, mask) 

        return mask 

    @staticmethod
    def _corner_angles(pts):
        angles = []
        n = len(pts)
        for i in range(n):
            p1 = pts[(i - 1) % n].astype(float)
            p2 = pts[i].astype(float)
            p3 = pts[(i + 1) % n].astype(float)
            v1 = p1 - p2
            v2 = p3 - p2
            cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
            angles.append(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0))))
        return angles

    def detect(self, mask, frame,
               min_area=1000,
               epsilon_ratio=0.08,
               aspect_tolerance=0.35,
               angle_tolerance=25):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for index, ctr in enumerate(contours):
            area = cv2.contourArea(ctr, oriented=False)
            if area < min_area:
                continue

            perimeter = cv2.arcLength(ctr, True)
            approx = cv2.approxPolyDP(ctr, epsilon_ratio * perimeter, True)

            if len(approx) != 4:
                continue
            if not cv2.isContourConvex(approx):
                continue

            # aspect ratio — square should be close to 1.0
            x, y, w, h = cv2.boundingRect(approx)
            aspect = float(w) / (h + 1e-6)
            if not (1.0 - aspect_tolerance < aspect < 1.0 + aspect_tolerance):
                continue

            # all corners should be ~90°
            pts = approx.reshape(-1, 2)
            angles = self._corner_angles(pts)
            if not all(90.0 - angle_tolerance < a < 90.0 + angle_tolerance for a in angles):
                continue

            self.rectangle = True
            M = cv2.moments(ctr)
            if M["m00"] != 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])
                centroids.append((u, v))
                cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)
                cv2.drawContours(frame, contours, index, (0, 255, 0), 2, cv2.LINE_8, hierarchy)
                cv2.putText(frame, f"{int(area)}", tuple(approx[0][0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        return frame, centroids
            


if __name__ == "__main__":
    cap = cv2.VideoCapture('video_2.mp4', cv2.CAP_FFMPEG)
    if not cap.isOpened(): 
        print("failed to open") 
        exit() 

    detectFrame = Detect(cap, False) 

    while True: 
        ret, frame = cap.read() 
        print("ret value: ", ret)
        if not ret: 
            print("Cannot receive frame")
            break
        cv2.imshow("video", frame)
        # redMask = detectFrame.maskRed(frame)
        blueMask = detectFrame.maskBlue(frame) 
        # redFrame = detectFrame.detect(redMask, frame)
        blueFrame = detectFrame.detect(blueMask, frame)
        cv2.imshow("video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()



    
