from audioop import avg
from ssl import CHANNEL_BINDING_TYPES
import cv2
import numpy as np
from sympy import im

# Import images 

def image2Blob_RGB (imgPath):
    img = cv2.imread(imgPath)

    shape = img.shape
    print("Image size: ", shape)
    
    #grayImg = np.zeros(shape, dtype=np.uint8)

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

def image2Blob_HSV (imgPath):
    img = cv2.imread(imgPath)

    # Convert RGB img to HSV
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

    shape = imgHsv.shape
    print("Image size: ", shape)
    
    # Split channels
    channel_h, channel_s, channel_v = cv2.split(imgHsv)
    # There is no need to convert to grayscale, since
    # the V channel already represents it 

    grayImg = channel_v

    # Threshold on the gray image
    ret, thresh1 = cv2.threshold(grayImg, 127, 255, cv2.THRESH_BINARY)

    # Threshold on the gray image
    # Take into account that OpenCV doesn't represent HUE on a 0-255 scale
    # The scale used is:
    # H: 0-179, S: 0-255, V: 0-255
    # https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
    hueThreshold = cv2.inRange(imgHsv,(1, 170, 20), (10, 255, 255) )
        # TODO: Find out an explanation for the V values

    # Make all images RGB, so we can plot them
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    thresh1 = cv2.cvtColor(thresh1, cv2.COLOR_GRAY2BGR)
    hueThreshold = cv2.cvtColor(hueThreshold, cv2.COLOR_GRAY2BGR)

    # Merge all images into a sigle one
    row1 = np.concatenate((imgHsv, grayImg), axis= 1)
    row2 = np.concatenate((thresh1, hueThreshold), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)

    cv2.imshow("Image changes", all) 

    cv2.waitKey(0)

def cleanNoise(imgPath):
    # https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html
    img = cv2.imread(imgPath)

    shape = img.shape
    print("Image size: ", shape)

    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    noisy = sp_noise(grayImg, 0.04)
    
    kernel = (7,7) # Kernel, how many pixels will be blured at the same time
    gausBlur = cv2.GaussianBlur(noisy, kernel, 0) # O std

    avgBlur = cv2.blur(noisy, kernel)

    # Merge all images into a sigle one
    row1 = np.concatenate((grayImg, noisy), axis= 1)
    row2 = np.concatenate((gausBlur, avgBlur), axis= 1)
    all = np.concatenate((row1, row2), axis= 0)

    cv2.imshow("Image noise", all)

    cv2.waitKey(0)

def sp_noise(image, prob):
    '''
    Add salt and pepper noise to image
    prob: Probability of the noise
    from: https://gist.github.com/lucaswiman/1e877a164a69f78694f845eab45c381a
    '''
    image = image.copy()
    if len(image.shape) == 2:
        black = 0
        white = 255            
    else:
        colorspace = image.shape[2]
        if colorspace == 3:  # RGB
            black = np.array([0, 0, 0], dtype='uint8')
            white = np.array([255, 255, 255], dtype='uint8')
        else:  # RGBA
            black = np.array([0, 0, 0, 255], dtype='uint8')
            white = np.array([255, 255, 255, 255], dtype='uint8')
    probs = np.random.random(image.shape[:2])
    image[probs < (prob / 2)] = black
    image[probs > 1 - (prob / 2)] = white
    return image

def addNoise2Gray(img):

    channel = img.copy()
    imgVar = np.var(channel)
    imgAvg = np.average(channel)
    
    noise = np.random.normal(imgAvg, imgVar, channel.shape)
    
    loosyChannel = noise + channel

    return loosyChannel

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
    
    #image2Blob_HSV(path)
    
    cleanNoise(path)


