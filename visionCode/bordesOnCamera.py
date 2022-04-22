import cv2
import numpy as np

#-------------------- Show video 

# Opens and reads the video file
cap = cv2.VideoCapture(0) #

# To actually show it, we should display it frame by frame
# To do this, We'll do a loop

while cap.isOpened():
    # Method tells you if the image was read sucessfully and the actual img
    readSuccess, frame = cap.read()
    
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    avgBlur = cv2.blur(grayImg, (7,7))
    imgCanny = cv2.Canny(avgBlur, 30, 30)

    # Draw contours:
    contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # Draw all contours
    # -1 signifies drawing all contours
                                        # Color
    cv2.drawContours(frame, contours, -1, (255, 0, 0), 3)
            

    # Merge all images into a sigle one
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    avgBlur = cv2.cvtColor(avgBlur, cv2.COLOR_GRAY2BGR)
    imgCanny = cv2.cvtColor(imgCanny, cv2.COLOR_GRAY2BGR)

    row1 = np.concatenate((frame, grayImg), axis= 1)
    row2 = np.concatenate((avgBlur, imgCanny), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)
    
    # Show frame
    cv2.imshow("All: ", all)

    # To stop the video before it ends 
    if cv2.waitKey(1) & 0xFF == ord ("q"):
        break
    

cap.release()
# Closes all windows
cv2.destroyAllWindows()


print("Done")

