import cv2
import numpy as np


def edgeDetection(imgPath, thresh1, thresh2):

    #TODO: Try with blured imgs

    img = cv2.imread(imgPath)
    #Edge detection with Canny func
                                # Threshold values
    imgCanny = cv2.Canny(img, thresh1, thresh2)

    # To make the borders THICK Bois
    kernel = np.ones((5,5), np.uint8)
        # more iterations -> thicker
    imgDialation = cv2.dilate(imgCanny, kernel, iterations = 1)

    # To make borders thiner
        # more iterations -> thinner
        # Since the first borders where already thin, we can eliminate them
        # if we set many interations
    imgEroded = cv2.erode(imgDialation, kernel, iterations = 1)

    # LaPlacian, another method to detect borders:
    # First, we apply a filter, to reduce the definition onf the img
    laplacianBlur = cv2.blur(img, (7,7))
    # Convert it to grayscale
    laplacianGray = cv2.cvtColor(laplacianBlur, cv2.COLOR_BGR2BGRA)
    #Apply Laplaciand filter. 
    imgLaplacian = cv2.Laplacian(laplacianGray, cv2.CV_64F)   

    # Make all images RGB, so we can plot them
    imgCanny = cv2.cvtColor(imgCanny, cv2.COLOR_GRAY2BGR)
    imgDialation = cv2.cvtColor(imgDialation, cv2.COLOR_GRAY2BGR)
    imgEroded = cv2.cvtColor(imgEroded, cv2.COLOR_GRAY2BGR)
    lap = np.zeros(laplacianBlur.shape, dtype = np.uint8)
    lap = imgLaplacian[:, :, 0:3]

    # imgLaplacian = imgLaplacian[:, :, 0:3]

    # Merge all images into a sigle one
    row1 = np.concatenate((img, imgCanny), axis= 1)
    row2 = np.concatenate((imgDialation, imgEroded), axis= 1)
    row3 = np.concatenate((laplacianBlur, lap), axis= 1)
    all = np.concatenate((row1, row3), axis= 0)


    cv2.imshow("Image changes", all)
    cv2.waitKey(0)

if __name__ == "__main__":
    path = r"./visionCode/sampleImages/building3.jpeg"
    
    #image2Blob_HSV(path)
    for t1 in range (40, 400, 20):
        for t2 in range (40, 400, 20):
            print(t1, t2)
            edgeDetection(path, t1, t2)
