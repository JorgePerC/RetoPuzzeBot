import cv2
print ("OpenCV imported")

#-------------------- SHow video 

# Opens and reads the video file
cap = cv2.VideoCapture(0) #

# To actually show it, we should display it frame by frame
# To do this, We'll do a loop
while True:
    # Method tells you if the image was read sucessfully and the actual img
    readSuccess, frame = cap.read()
    
    # Show frame
    cv2.imshow("Video Feed", frame)

    # To stop the video before it ends 
    if cv2.waitKey(1) & 0xFF == ord ("q"):
        break

# Closes all windows
cv2.destroyAllWindows()


print("Done")
