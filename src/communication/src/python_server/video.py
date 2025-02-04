import threading
import numpy as np
import time
import cv2
from collections import deque
from cv_bridge import CvBridge


class VideoConnection:
    def __init__(self, udp_socket=None, encoding=None):
        if udp_socket is not None:
            self.socket = udp_socket
            self.encoding = encoding
            self.buffer = deque([], 100)
            self.frame = None
            self.MAX_DGRAM = 65507
            threading.Thread(target=self.receive, daemon=True).start()
            threading.Thread(target=self.process, daemon=True).start()

    def receive(self):
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) == 0:
                    continue
                self.buffer.append(seg)
            except Exception as e:
                print(e)

    def process(self):
        while True:
            if len(self.buffer) > 0:
                self.frame = self.parse_image(self.buffer.popleft())
            time.sleep(0.008)

    def parse_image(self, bytes):
        try:
            if len(bytes) != 0 and self.encoding == 'bgr8':
                np_array = np.frombuffer(bytes, dtype=np.uint8)
                cv_image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
                bridge = CvBridge()
                return bridge.cv2_to_imgmsg(cv_image, encoding=self.encoding)
            elif len(bytes) != 0 and self.encoding == '32FC1':
                np_array = np.frombuffer(bytes, dtype=np.uint8)
                cv_image = cv2.imdecode(np_array, cv2.IMREAD_UNCHANGED)
                cv_image = (cv_image).astype(np.uint16)
                bridge = CvBridge()
                return bridge.cv2_to_imgmsg(cv_image, encoding='mono16')
            return None
        except Exception as e:
            print(e)
