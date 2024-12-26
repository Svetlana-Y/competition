import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from module.logger import put_log
from module.config import (
    LINES_H_RATIO,
    )

def dark_tunnel(follow_trace, img):
    if follow_trace.sleep:
        follow_trace.sleep =0
        time.sleep(3.0)
        put_log(follow_trace, "Сладкий слип")
    message = Twist()
    
    scan_data = follow_trace.lidar_data.ranges 
    front = min(scan_data[0:10] + scan_data[352:359])
    left = min(scan_data[83:100])
    right = min(scan_data[265:280])
    yaw = follow_trace.pose.pose.pose.orientation.z  # Текущий угол робота

    put_log(follow_trace, f"Avoidance level: {follow_trace.avoidance}")
    put_log(follow_trace, f"Front: {front}, Left: {left}, Right: {right}")

    if follow_trace.avoidance == 0:
        put_log(follow_trace, message="Начало движения прямо")
        follow_trace.avoidance = 1 

    if follow_trace.avoidance == 1:
        message.linear.x = 0.2  
        message.angular.z = 0.0 
        if front < 0.2: 
            follow_trace.avoidance = 2  
            follow_trace.target_yaw = yaw + 0.715  
            put_log(follow_trace, message="Препятствие впереди, начинаем поворот налево")

   
    if follow_trace.avoidance == 2:
        message.linear.x = 0.0  
        message.angular.z = 0.3  
        if yaw >= follow_trace.target_yaw - 0.05:  
            follow_trace.avoidance = 3  
            put_log(follow_trace, message="Поворот налево завершен, движение прямо")

    if follow_trace.avoidance == 3:
        message.linear.x = 0.2
        message.angular.z = 0.0
        if right > 2.0:
            follow_trace.avoidance = 4 
            follow_trace.target_yaw = yaw - 0.715  
            put_log(follow_trace, message="Препятствие справа исчезло, начинаем поворот направо")

    # Поворот направо на 90 градусов
    if follow_trace.avoidance == 4:
        message.linear.x = 0.0 
        message.angular.z = -0.3
        if yaw <= follow_trace.target_yaw + 0.05:
            follow_trace.avoidance = 5 
            put_log(follow_trace, message="Поворот направо завершен, финальное движение прямо")

    # Финальное движение прямо
    if follow_trace.avoidance == 5:
        message.linear.x = 0.2
        message.angular.z = 0.0

    if (front > 100.0 and left > 100.0 and right > 100.0) or (left > 100.0 and front > 100.0 and follow_trace.avoidance == 4):
        message.angular.z = 0.0
        message.linear.x = 0.0
        follow_trace._linear_speed = 0
        put_log(follow_trace, message="Задание завершено, робот остановлен")
        msg = String()
        msg.data = "one_1"
        follow_trace._sign_finish.publish(msg)

    follow_trace._robot_cmd_vel_pub.publish(message)
