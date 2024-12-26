# парковка
import time
from geometry_msgs.msg import Twist

from module.logger import put_log
from collections import namedtuple

def determine_parking_side(follow_trace, yaw, dist):
    word = ['влево', 'вправо']
    if yaw < 0.8 and (dist.left < 0.5 or dist.right < 0.5):
        put_log(follow_trace, message=f"паркуемся {word[follow_trace.parking.side]}")
        follow_trace.parking.state = 1
        if dist.left < 0.5:
            follow_trace.parking.side = 1

def start_parking(follow_trace, yaw, message):
    word = ['левую', 'правую']
    put_log(follow_trace, message=f"Заезжаю на {word[follow_trace.parking.side]} парковку")     
    need_yaw = [lambda x: x < 0.35, lambda x: x > 0.90]

    if follow_trace.parking.side == 1:
        message.angular.z = -2.0
        message.linear.x = 0.35
    else:
        message.angular.z = 2.0
        message.linear.x = 0.3

    if need_yaw[follow_trace.parking.side](yaw):
        follow_trace.parking.state = 2
        message.angular.z = 0.0
        message.linear.x = 0.0


def end_parking(follow_trace, dist, message):
    word = ['левой', 'правой']
    put_log(follow_trace, message=f"Выезжаю c {word[follow_trace.parking.side]} парковки")
    need_dist = [lambda x: x.left <= 0.4, lambda x: x.right <= 0.35 or x.front <= 1]
    
    if follow_trace.parking.side == 1:
        message.angular.z = -0.8
        message.linear.x = -0.02
    else:
        message.angular.z = 2.0
        message.linear.x = -0.13

    if need_dist[follow_trace.parking.side](dist):
        follow_trace.parking.state = 4
        message.angular.z = 0.4 if follow_trace.parking.side else 0.03
        message.linear.x = follow_trace._linear_speed/2

def parking(follow_trace):
    message = Twist()
    Distances = namedtuple('Distances', ['front', 'left', 'right'])

    scan_data = follow_trace.lidar_data.ranges
    dist = Distances(min(scan_data[0:20] + scan_data[340:359]), min(scan_data[50:80]), min(scan_data[270:300]))
    yaw = follow_trace.pose.pose.pose.orientation.z

    if follow_trace.parking.state == 0:
        determine_parking_side(follow_trace, yaw, dist)

    if follow_trace.parking.state == 1:
        start_parking(follow_trace, yaw, message)

    if follow_trace.parking.state == 3:
        end_parking(follow_trace, dist, message)


    if follow_trace.parking.state: 
        follow_trace._robot_cmd_vel_pub.publish(message)   

        if follow_trace.parking.state == 2: 
            put_log(follow_trace, message=f"стоим")
            time.sleep(3.0)
            put_log(follow_trace, message=f"выезжаем")
            follow_trace.parking.state = 3
        
        if follow_trace.parking.state == 4:
            time.sleep(3.0)
            follow_trace.parking.state = 0
            follow_trace.TASK_LEVEL = 3.5
            follow_trace.next_goal = 1
            follow_trace.finish_parking = time.time()
            put_log(follow_trace, message=f"Миссия выполнена")
