# препятствия

import cv2
import numpy as np
from module.yolo import find_signs

from geometry_msgs.msg import Twist

from module.config import (
    LINES_H_RATIO,
    )

from module.logger import put_log


def check_yellow_color(perspectiveImg_, middle_h = None):
    _, width, _ = perspectiveImg_.shape
    left_half = perspectiveImg_[:, :width//2, :]

    h, _, _ = left_half.shape
    if middle_h is None:
        middle_h = int(h * LINES_H_RATIO)
    yellow_range_mask = cv2.inRange(left_half, (0, 240, 255), (0, 255, 255))
    
    dilated_mask = cv2.dilate(yellow_range_mask, np.ones((2, 2)), iterations=4)
    return dilated_mask

def check_white_color(perspectiveImg_, middle_h = None):
    _, width, _ = perspectiveImg_.shape
    right_half = perspectiveImg_[:, width//2:, :]

    h, _, _ = right_half.shape
    if middle_h is None:
        middle_h = int(h * LINES_H_RATIO)

    white_range_mask = cv2.inRange(right_half, (250, 250, 250), (255, 255, 255))

    return white_range_mask

def avoid_walls(follow_trace, img):
    message = Twist()
    
    perspective = follow_trace._warpPerspective(img)
    perspective_h, _, _ = perspective.shape

    hLevelLine = int(perspective_h*LINES_H_RATIO)

    yellow_mask = check_yellow_color(perspective, hLevelLine)
    top_half = yellow_mask[:int(len(yellow_mask[0])/1.325),:]
    down_half = yellow_mask[int(len(yellow_mask[0])/1.325):,:]

    white_mask = check_white_color(perspective, hLevelLine)
    
    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:10]+scan_data[349:359])

    if front < 0.45:
        put_log(follow_trace, message=f"Обнаружено препятствие впереди, поворачиваем")
        
        message.linear.x = 0.0
        message.angular.z = -0.5 if 1.5 <= follow_trace.avoidance <= 2 else 0.5
        follow_trace.avoidance = 1 if follow_trace.avoidance < 1 else follow_trace.avoidance
        
    elif cv2.countNonZero(top_half) < cv2.countNonZero(down_half) and 1 <= follow_trace.avoidance <= 1.5:
        message.linear.x = 0.0
        message.angular.z = -0.5
        follow_trace.avoidance = 1.5
        
    elif follow_trace.avoidance == 1.5 and cv2.countNonZero(top_half) == 0 and cv2.countNonZero(down_half)==0:
        follow_trace.avoidance = 2
        
    elif follow_trace.avoidance:
        down = white_mask[int(len(white_mask[0])/2):,:]
        top = white_mask[:int(len(white_mask[0])/2),:]

        if follow_trace.avoidance == 2 and (cv2.countNonZero(down) > cv2.countNonZero(top)):
            put_log(follow_trace, message=f"Миссия выполнена")
            follow_trace.avoidance = 0
            follow_trace.TASK_LEVEL = 2.5
            follow_trace.next_goal = 5
        else:
            put_log(follow_trace, message=f"Препятствий не обнаружено")
            message.linear.x = follow_trace._linear_speed
            message.angular.z = 0.0

    if follow_trace.avoidance: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
