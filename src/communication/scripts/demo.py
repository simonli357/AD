from server import Server
import time
import cv2
from cv_bridge import CvBridge

# Initialize server
server = Server(49153)
server.initialize()
bridge = CvBridge()

while True:
    if server.get_client().strings:
        print(server.get_client().strings.pop(0))

    if server.get_client().images:
        img_msg = server.get_client().images.pop(0)

        cv_image = bridge.imgmsg_to_cv2(img_msg)

        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)

    # Check for array messages
    if server.get_client().arrays:
        arr_msg = server.get_client().arrays.pop(0)
        print("Received array")

    # Check for other messages
    if server.get_client().messages:
        msg = server.get_client().messages.pop(0)
        print(f"{msg.data}")

    time.sleep(0.01)

cv2.destroyAllWindows()
