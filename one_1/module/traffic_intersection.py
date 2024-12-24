# развилка
from module.logger import put_log
from module.yolo import find_signs

def check_direction(follow_trace, img):
    recognised_signs = find_signs(follow_trace, img, treshold = 10000)
    if 4 in recognised_signs.cls:
        put_log(follow_trace, "[Перекресток] Поворот налево")
        follow_trace.MAIN_LINE = "YELLOW"
        follow_trace.TASK_LEVEL = 1.5
    elif 7 in recognised_signs.cls:
        put_log(follow_trace, "[Перекресток] Поворот направо")
        follow_trace.MAIN_LINE = "WHITE"
        follow_trace.TASK_LEVEL = 1.5
                          
