import cv2
import numpy as np

def eccentricity_from_moments(moments):
    """Calculates the eccentricity from the moments of the contour"""

    a1 = (moments['mu20'] + moments['mu02']) / 2
    a2 = np.sqrt(4 * moments['mu11'] ** 2 + (moments['mu20'] - moments['mu02']) ** 2) / 2
    ecc = np.sqrt(1 - (a1 - a2) / (a1 + a2))
    return ecc

def compactness_from_moments(moments, contour):
    length = cv2.arcLength(contour, True)
    k = (length * length) / (moments['m00'] * 4 * np.pi)
    return k

def direction_from_moments(moments):
    # https://docs.baslerweb.com/visualapplets/files/manuals/content/examples%20imagemoments.html

    x_mean = moments["m10"] / moments["m00"]
    y_mean = moments["m01"] / moments["m00"]

    u_11 = moments['m11']/moments['m00'] - (x_mean*y_mean)
    u_02 = moments['m02']/moments['m00'] - y_mean**2
    u_20 = moments['m20']/moments['m00'] -  x_mean**2
    a = 2*u_11
    b = u_02 + u_20
    angle = 0.5*np.arctan2(a, b)
    return np.degrees(angle)

# Image has already been thresholded
image = cv2.imread('../../fotosCurva2/fotoSeg93.jpg') #foto9, foto56, fotosCurva2/fotoSeg93, fotoSeg201, fotoSeg72, fotoSeg102

ret, image = cv2.threshold(image, 127,255,cv2.THRESH_BINARY)
cv2.imshow("Original", image)
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
image = cv2.dilate(image, (9,9), iterations=10)
image = cv2.equalizeHist(image)
image = cv2.blur(image, (9,9))

# canny = cv2.Canny(image, 100, 300)

# cv2.imshow("Canny", canny)

contours, hierarchy = cv2.findContours(image, method=cv2.CHAIN_APPROX_SIMPLE, mode=cv2.RETR_EXTERNAL)
print("Totoal contours", len(contours))
print("=========")
result = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

cv2.drawContours(result, contours, contourIdx=-1, color=(250, 0, 0), thickness=1, lineType=cv2.LINE_AA)

c = 0
# Calculate moments
for cont in contours:
    M = cv2.moments(cont)
    for i in M:
        if M[i] == 0:
            M[i] += 1e-5
    if M['m00'] < 1000:
        continue
    print("Contour area: '{}'".format(M['m00']))

    print("center X : '{}'".format(round(M['m10'] / M['m00'])))
    print("center Y : '{}'".format(round(M['m01'] / M['m00'])))

    ecc = eccentricity_from_moments(M)
    comp = compactness_from_moments(M, cont)
    dir = direction_from_moments(M)

    

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    cv2.circle(result, (cX, cY), 5, (255, 0, 0), -1)
    cv2.putText(result, "{}_cen".format(c), (cX - 25, cY + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    if (ecc > 0.94):
        x, y, w, h = cv2.boundingRect(cont)
        cv2.rectangle(result, (x,y), (x+w,y+h), (0,255,0))
        # display the image
        print("Fig. {} | Ecc:".format(c), ecc, "| Compactness:", comp, "| Direction: ", dir)
    cv2.imshow("Image", result)
    c+=1
    print("==--==")

cv2.waitKey(0)
cv2.destroyAllWindows()







# def roundness(contour, moments):
#     """Calculates the roundness of a contour"""

#     length = cv2.arcLength(contour, True)
#     k = (length * length) / (moments['m00'] * 4 * np.pi)
#     return k

# def eccentricity_from_ellipse(contour):
#     """Calculates the eccentricity fitting an ellipse from a contour"""

#     (x, y), (MA, ma), angle = cv2.fitEllipse(contour)

#     a = ma / 2
#     b = MA / 2

#     ecc = np.sqrt(a ** 2 - b ** 2) / a
#     return ecc



# # Compute Hu moments:
# HuM = cv2.HuMoments(M)
# print("Hu moments: '{}'".format(HuM))

# #  Find contours
# contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# # Compute moments:
# M2 = cv2.moments(contours[0])
# print("moments: '{}'".format(M2))

# # Calculate the centroid of the contour based on moments:
# x2, y2 = centroid(M2)

# # Compute Hu moments:
# HuM2 = cv2.HuMoments(M2)
# print("Hu moments: '{}'".format(HuM2))



# def sort_contours_size(cnts):
#     """ Sort contours based on the size"""

#     cnts_sizes = [cv2.contourArea(contour) for contour in cnts]
#     (cnts_sizes, cnts) = zip(*sorted(zip(cnts_sizes, cnts)))
#     return cnts_sizes, cnts

# coordinate = ['x', 'y', 'z']
# value = [5, 4, 3]
# result = zip(coordinate, value)
# print(list(result))
# c, v =  zip(*zip(coordinate, value))
# print('c =', c)
# print('v =', v)

# [('x', 5), ('y', 4), ('z', 3)]
# c = ('x', 'y', 'z')
# v = (5, 4, 3)


# coordinate = ['x', 'y', 'z']
# value = [5, 4, 3]
# print(sorted(zip(value, coordinate)))
# c, v = zip(*sorted(zip(value, coordinate)))
# print('c =', c)
# print('v =', v)

# [(3, 'z'), (4, 'y'), (5, 'x')]
# c = (3, 4, 5)
# v = ('z', 'y', 'x')

# rotated_rect = cv2.minAreaRect(contours[0])



