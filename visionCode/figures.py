import cv2
import numpy as np

cap = cv2.VideoCapture(0) # , cv2.CAP_DSHOW

while cap.isOpened():

    ret, frame = cap.read()

    origin = frame.copy()
    kernel = np.ones((7,7), np.uint8)

    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    bordesCanny = cv2.Canny(frame, 100, 200)

    # We need to erode the image bc if don't, then, the circle won't
    # be detected
    bordesCanny =  cv2.dilate(bordesCanny, kernel, iterations=1)
    # SOmething to try: from the original gray image, restar the cannycorder
    # May give a better result. 

    circles = cv2.HoughCircles(bordesCanny, 
                            cv2.HOUGH_GRADIENT_ALT, # for better accuracy
                            1, # for evalualuating the image on same dimentions
                            minDist=5, # Distance between circle center, if smaller, circle is discarted   
                            param1=300, # Dunno https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
                            param2=0.9, # 0-1, how much of a circle?
                            minRadius=5 , maxRadius=0)

    try: 
        circles = np.uint16(np.around(circles))

        for i in circles[0, :]:
            # dibujar circulo
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 3)
            # dibujar centro
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        
        _, threshold = cv2.threshold(grayImg, 110, 255,
                                    cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 400:
                approx = cv2.approxPolyDP(cnt,
                                        0.009 * cv2.arcLength(cnt, True), True)

                if (len(approx) == 7):
                    cv2.drawContours(grayImg, [approx], 0, (0, 0, 255), 5)
    except:
        #print("Not enough circles where found")
        pass

    # Merge all images into a sigle one
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    bordesCanny = cv2.cvtColor(bordesCanny, cv2.COLOR_GRAY2BGR)


    row1 = np.concatenate((origin, grayImg), axis= 1)
    row2 = np.concatenate((bordesCanny, frame), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)

    cv2.imshow("Frame", all)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()