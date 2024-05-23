import cv2
import numpy as np

img = cv2.imread("/home/autoware/gokart-autoware/src/universe/autoware.universe/gokart/Perception/classic_grass_detection/images/box.png")
img = cv2.blur(img, (1, 1))
b,g,r = cv2.split(img)
grass_gray = cv2.absdiff(0.3 * g.astype(np.float64), 1 * b.astype(np.float64))
cone_gray = cv2.absdiff(r, b)
#grass_only = grass_gray.copy()
#cv2.subtract(grass_gray.astype(np.float64), cone_gray.astype(np.float64), grass_only)
ret, grass2 = cv2.threshold(grass_gray, 100, 255, cv2.THRESH_BINARY)

#kernel = np.ones((5, 5),np.uint8)
#grass = cv2.morphologyEx(grass_only, cv2.MORPH_OPEN, kernel)
#grass = cv2.morphologyEx(grass_only, cv2.MORPH_CLOSE, kernel)

cv2.imwrite("/home/autoware/Pictures/Screenshots/r1.png", grass_gray)
cv2.imwrite("/home/autoware/Pictures/Screenshots/r2.png", cone_gray)
cv2.imwrite("/home/autoware/Pictures/Screenshots/r3.png", grass2)
