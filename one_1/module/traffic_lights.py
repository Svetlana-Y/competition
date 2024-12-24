# светофор

import cv2
import numpy as np

from module.logger import put_log

import cv2
import numpy as np

def check_green_color(image):

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    min_green = np.array([40, 50, 50])   
    max_green = np.array([80, 255, 255]) 
    
    green_area = cv2.inRange(hsv_image, min_green, max_green)
    return np.count_nonzero(green_area) > 0


def check_traffic_lights(follow_trace, img):
    if check_green_color(img):
        put_log(follow_trace, "Поехали")
        follow_trace.STATUS_CAR = 1
        follow_trace.TASK_LEVEL = 1
    else:
        put_log(follow_trace, "Ждём зеленый сигнал")
