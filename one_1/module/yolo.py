from module.logger import put_log
from collections import namedtuple

names = ['construction', 'crossing', 'green', 'intersection', 'left', 'parking', 'red_yellow', 'right', 'tunnel']


def find_signs(follow_trace, img, treshold = 10000):
    detect = namedtuple('detect', ['cls', 'area'])
    finds = detect([], [])
    results = follow_trace.model(img, conf=0.85, verbose=False)
    for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                area = (x2 - x1) * (y2 - y1)
                if (area > treshold):
                    finds.cls.append(box.cls)
                    finds.area.append(area)
    return finds

def find_target_sign(follow_trace, img, treshold=10000):
    recognised_signs = find_signs(follow_trace, img, treshold = treshold)
    if follow_trace.next_goal in recognised_signs.cls:
        put_log(follow_trace, f"{names[follow_trace.next_goal]} detected")
        follow_trace.TASK_LEVEL += 0.5
