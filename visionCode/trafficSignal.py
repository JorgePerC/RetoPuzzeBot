import cv2
import numpy as np

img = cv2.imread('road57.png', 1)

while True:
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    ret, thresh = cv2.threshold(imgGray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((7, 7), np.uint8)
    erosion = cv2.erode(imgGray, kernel, iterations=1)
    apertura = cv2.morphologyEx(imgGray, cv2.MORPH_OPEN, kernel)
    cierre = cv2.morphologyEx(imgGray, cv2.MORPH_CLOSE, kernel)
    cv2.imshow('image', thresh)
    cv2.imshow('erosion', erosion)
    cv2.imshow('apertura', apertura)
    cv2.imshow('cierre', cierre)

    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    background = cv2.dilate(opening, kernel, iterations=3)
    distance_transform = cv2.distanceTransform(opening, cv2.DIST_L2, maskSize=5)
    _, foreground = cv2.threshold(distance_transform, 0.7 * distance_transform.max(), 255, 0)
    foreground = np.uint8(foreground)
    unknown = cv2.subtract(background, foreground)
    _, markers = cv2.connectedComponents(foreground)
    markers = markers + 1
    markers[unknown == 255] = 0
    markers = cv2.watershed(img, markers)
    img[markers == -1] = [0, 0, 255]
    cv2.imshow('aperturaWater', img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()