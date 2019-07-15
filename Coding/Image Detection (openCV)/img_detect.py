import cv2
import sys
import numpy as np


frame = cv2.imread('t1.jpg')

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#edges = cv2.Canny(gray,50,100,apertureSize = 3)

lower_color_range = np.array([15, 65, 90])
upper_color_range = np.array([25, 210, 220])
mask = cv2.inRange(hsv, lower_color_range, upper_color_range)
#mask = cv2.subtract(255, maskm) 

_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)

for contour in contours:
        area = cv2.contourArea(contour)
 
        if area > 20000:
            cv2.drawContours(frame, contour, -1, (0, 255, 0), 30)


#cv2.drawContours(frame, contours, -1, (0, 255, 0), 20)

# Display the resulting frame
cv2.imwrite('1a.jpg',frame)
cv2.imwrite('1b.jpg',mask)
