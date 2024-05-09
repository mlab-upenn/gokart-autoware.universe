import cv2
import numpy as np

img = cv2.imread("/home/autoware/Pictures/Screenshots/calibration.jpg")
img = cv2.blur(img, (10, 10))
b,g,r = cv2.split(img)
grass_gray = cv2.absdiff(g, b)
cone_gray = cv2.absdiff(r, b)
grass_only = grass_gray.copy()
cv2.subtract(grass_gray, cone_gray, grass_only)
ret, grass = cv2.threshold(grass_only, 1, 255, cv2.THRESH_BINARY)
kernel = np.ones((5, 5),np.uint8)
grass = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, kernel)
grass = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, kernel)

cv2.imwrite("/home/autoware/Pictures/Screenshots/r1.png", grass_gray)
cv2.imwrite("/home/autoware/Pictures/Screenshots/r2.png", cone_gray)
cv2.imwrite("/home/autoware/Pictures/Screenshots/r3.png", grass_only)
