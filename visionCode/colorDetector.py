import cv2
import numpy as np
import time

#cap = cv2.VideoCapture('videoSemaforo2.mp4')
img= cv2.imread('semaforo.png', 1)

redBajo1 = np.array([0, 100, 20], np.uint8)
redAlto1 = np.array([8, 255, 255], np.uint8)

redBajo2=np.array([175, 100, 20], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

greenBajo=np.array([35, 100, 20], np.uint8)
greenAlto=np.array([82, 255, 255], np.uint8)

yellowBajo = np.array([15, 100, 20], np.uint8)
yellowAlto = np.array([40, 255, 255], np.uint8)
while True:
    #ret, frame = cap.read()
    time.sleep(0.03)
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV, greenBajo, greenAlto)
    mask2 = cv2.inRange(frameHSV, yellowBajo, yellowAlto)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    maskRedvis = cv2.bitwise_and(img, img, mask=maskRed)
    maskYellowvis = cv2.bitwise_and(img, img, mask=mask2)
    maskGreenvis = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow('frame', img)
    cv2.imshow('Green', maskGreenvis)
    cv2.imshow('Yellow', maskYellowvis)
    cv2.imshow('Red', maskRedvis)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
      break
cap.release()
cv2.destroyAllWindows()