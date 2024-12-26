# медленный пешеход

import cv2
import numpy as np
from geometry_msgs.msg import Twist

from module.config import (
    LINES_H_RATIO,
    )

from module.logger import put_log


def check_line(follow_trace, perspectiveImg_, middle_h = None):
    h_, w_, _ = perspectiveImg_.shape
    perspectiveImg = perspectiveImg_[:, 350:, :]

    h, w, _ = perspectiveImg.shape
    middle_h = int(h * LINES_H_RATIO) if middle_h is None else middle_h

    yellow_mask = cv2.inRange(perspectiveImg, (20, 40, 70), (60,80,90))
    yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)
    return yellow_mask
    
def walker(follow_trace, img):
    
    perspective = follow_trace._warpPerspective(img)
    hLevelLine = int(perspective.shape[0]*LINES_H_RATIO)
    yellow_mask = check_line(follow_trace, perspective, hLevelLine)

    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:15]+scan_data[340:359])
    
    if not follow_trace.pedestrian_started:
        line = cv2.countNonZero(yellow_mask) > 0
        if line:
            put_log(follow_trace, "Желтая линия обнаружена")
            follow_trace.MAIN_LINE = "WHITE"
            follow_trace.pedestrian_started = True
    else:
        line = True
        
    lidar_data = follow_trace.lidar_data.ranges
    if  front > 0.7:
        put_log(follow_trace, "Гоним")
        follow_trace._linear_speed = 0.25
        message = Twist()
        message.linear.x = follow_trace._linear_speed
        follow_trace._robot_cmd_vel_pub.publish(message)
        follow_trace.TASK_LEVEL = 4.5
        follow_trace.next_goal = 8
    else:
        put_log(follow_trace, "ПУПУПУ ждем медленного пешехода")
        message = Twist()
        follow_trace._linear_speed = 0.0
        message.linear.x = 0.0
        message.angular.z = 0.0
        follow_trace._robot_cmd_vel_pub.publish(message)
        
