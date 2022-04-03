import cv2
import numpy as np
print ("OpenCV imported")

#-------------------- SHow video 

# Opens and reads the video file
cap = cv2.VideoCapture(0) #

# To actually show it, we should display it frame by frame
# To do this, We'll do a loop

while cap.isOpened():
    # Method tells you if the image was read sucessfully and the actual img
    readSuccess, frame = cap.read()
    shape = frame.shape
    # Split image channels
    channel_b, channel_g, channel_r = cv2.split(frame)

    red_img = np.zeros(shape, dtype=np.uint8)
    green_img = np.zeros(shape, dtype=np.uint8)
    blue_img = np.zeros(shape, dtype=np.uint8)

    #assign the red channel of src to empty image
    red_img[:,:,2] = channel_r
    green_img[:,:,1] = channel_g
    blue_img[:,:,0] = channel_b
        

    # Merge all images into a sigle one
    row1 = np.concatenate((frame, blue_img), axis= 1)
    row2 = np.concatenate((green_img, red_img), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)
    
    # Show frame
    cv2.imshow("All: ",all)

    # To stop the video before it ends 
    if cv2.waitKey(1) & 0xFF == ord ("q"):
        break
    

cap.release()
# Closes all windows
cv2.destroyAllWindows()


print("Done")

