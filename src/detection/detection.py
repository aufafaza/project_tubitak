import cv2
import numpy as np

class Detect: 
    videoCapture = None;
    rectangle = False;
    
    def __init__(self, videoSource: cv2.VideoCapture, rectangle: bool): 
        self.videoCapture = videoSource;  
        self.rectangle = rectangle; 

    def maskRed(self, bgr: cv2.typing.MatLike): 
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

    def maskBlue(self, bgr: cv2.typing.MatLike): 
        blur = cv2.GaussianBlur(bgr, (7, 7), 0)
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

    def detect(self, mask: cv2.typing.MatLike, frame: cv2.typing.MatLike):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for index, ctr in enumerate(contours): 
            area = cv2.contourArea(ctr, oriented=False)
            if (area < 500): continue 
            perimeter = cv2.arcLength(ctr, True)
            epsilon = 0.03 * perimeter 

            approx = cv2.approxPolyDP(ctr, epsilon, True)
            
            edges = len(approx) 

            if (edges == 4): 
                self.rectangle = True
                label = str(area)
            else: 
                label = "not found" 
            

            text_coords = tuple(approx[0][0])
            cv2.drawContours(frame, contours, index, (0, 255, 0), 2, cv2.LINE_8, hierarchy)
            cv2.putText(frame, label, text_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            


if __name__ == "__main__":
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
    print(cap.getBackendName)
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
        redMask = detectFrame.maskRed(frame)
        blueMask = detectFrame.maskBlue(frame) 
        redFrame = detectFrame.detect(redMask, frame)
        blueFrame = detectFrame.detect(blueMask, frame)
        cv2.imshow("video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
     


    
