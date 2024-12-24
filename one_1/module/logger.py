from collections import deque

previous_message = ''

def put_log(node, message):
    global previous_message_id, previous_message
    if message == previous_message:
        return
    
    message_ = f"--- {message}"
    
    
    node.get_logger().info(message_)
    previous_message = message

