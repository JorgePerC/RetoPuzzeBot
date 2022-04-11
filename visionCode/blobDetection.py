import cv2
import numpy as np
from sympy import im

# Import images 

def image2Blob (imgPath):
    img = cv2.imread(imgPath)

    shape = img.shape
    print("Image size: ", shape)
    
    grayImg = np.zeros(shape, dtype=np.uint8)

    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh1 = cv2.threshold(grayImg, 127, 255, cv2.THRESH_BINARY)

    channel_b, channel_g, channel_r = cv2.split(img)
    res, channelThreshold = cv2.threshold(channel_b, 127, 255, cv2.THRESH_BINARY)
    
    #Make all images RGB, so we can plot them
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    thresh1 = cv2.cvtColor(thresh1, cv2.COLOR_GRAY2BGR)
    channelThreshold = cv2.cvtColor(channelThreshold, cv2.COLOR_GRAY2BGR)

    # Merge all images into a sigle one
    row1 = np.concatenate((img, grayImg), axis= 1)
    row2 = np.concatenate((thresh1, channelThreshold), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)

    cv2.imshow("Image changes", all) 

    cv2.waitKey(0)

def addNoise(img):
    
    for i in range( img.shape[3]):
        channel =  img[:,:,i]
        imgVar = np.var(channel)
        imgAvg = np.average(channel)
        
        noise = np.random.normal(imgAvg, .1, img.shape)
        
        loosyChannel = noise + channel
        # Save into the original channel
        img[:,:,i] = loosyChannel

    return img

if __name__ == "__main__":
    path = r"./visionCode/sampleImages/road47.png"

    image2Blob(path)


