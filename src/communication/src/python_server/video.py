import threading
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import OrderedDict


class VideoConnection:
    def __init__(self, client_socket=None, encoding=None):
        if client_socket is not None:
            self.socket = client_socket
            self.encoding = encoding
            self.frame = None
            self.MAX_DGRAM = 65507
            self.MAX_SIZE = 921667
            self.REST = self.MAX_SIZE - ((self.MAX_DGRAM - 1) * 14)
            self.bytes_table = OrderedDict({
                2: b'',
                1: b'',
            })
            threading.Thread(target=self.receive, daemon=True).start()
            threading.Thread(target=self.update, daemon=True).start()

    def receive(self):
        while True:
            seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
            seg_num = seg[0]
            bytes = seg[1:]
            if len(bytes) == 0 or seg_num not in self.bytes_table.keys():
                continue
            self.bytes_table[seg_num] = bytes

    def update(self):
        while True:
            bytes = b''
            for key, value in self.bytes_table.items():
                bytes += value
            self.frame = self.parse_image(bytes)
            time.sleep(0.016)

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
                if cv_image is None:
                    print("Failed to decode the image. The data might be corrupted.")
                    return None
                # cv_image = cv_image.astype(np.float32)
                cv_image = (cv_image).astype(np.uint16)
                bridge = CvBridge()
                return bridge.cv2_to_imgmsg(cv_image, encoding='mono16')
            return None
        except Exception as e:
            print(e)
