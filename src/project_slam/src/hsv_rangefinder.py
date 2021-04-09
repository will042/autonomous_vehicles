#Use this for determining threshold for color detection

import cv2
import numpy as np
img = cv2.imread(test_img)

#Helper function to set size & position of image windows

resize_factor = 1

def show_img(window_name, img):
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    h,w = img.shape[:2]
    h = int(h / resize_factor)
    w = int(w / resize_factor)
    cv2.resizeWindow(window_name, w, h)
    cv2.moveWindow(window_name, *(50,50))
    cv2.imshow(window_name, img)

show_img("Input Image", img)


red = np.uint8([[[0,16,79 ]]])
hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
print( hsv_red )


hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

show_img('hsv',hsv)

red1 = (160.0, 100.0, 100.0)
red2 = (179.0, 255.0, 255.0)

mask = cv2.inRange(hsv, red1, red2)

result = cv2.bitwise_and(hsv, hsv, mask=mask)

# hits = (red_channel > threshold

# count = (hits == 1).sum()
# x_center, y_center = np.argwhere(hits==1).sum(0)/count

cv2.imshow('mask', mask)
cv2.imshow('result', result)

cv2.waitKey(0)
cv2.destroyAllWindows()