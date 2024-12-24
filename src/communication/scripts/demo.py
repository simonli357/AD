from server import Server
import time
import cv2
from cv_bridge import CvBridge

server = Server(49153)
server.initialize()
server.get_client().send_string("TEST")  # String test
bridge = CvBridge()

while True:
    if server.get_client().strings:
        print(server.get_client().strings.pop(0))
    if server.get_client().images:
        img_msg = server.get_client().images.pop(0)
        print(f"Received Image - Height: {img_msg.height}, Width: {img_msg.width}")
        cv_image = bridge.imgmsg_to_cv2(img_msg)
        cv2.imwrite("./image.png", cv_image)
    if server.get_client().arrays:
        arr_msg = server.get_client().arrays.pop(0)
        print("Received array")
    if server.get_client().messages:
        msg = server.get_client().messages.pop(0)
        print(f"{msg.data}")

    time.sleep(1)
