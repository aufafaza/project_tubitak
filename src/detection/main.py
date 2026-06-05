import cv2
if __name__ == "__main__":
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
    print(cap.getBackendName)
    if not cap.isOpened(): 
        print("failed to open") 
        exit() 
    
    # detectFrame = Detect(cap, False) 
    
    while True: 
        ret, frame = cap.read() 
        print("ret value: ", ret)
        if not ret: 
            print("Cannot receive frame")
            break
        cv2.imshow("video", frame)
        # redMask = detectFrame.colorDetectBlue(frame)
        # blueMask = detectFrame.colorDetectBlue(frame) 
        # cv2.imshow("redMask", redMask)
        # cv2.imshow("blueMask", blueMask)
        # detectFrame.detect(redMask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    
