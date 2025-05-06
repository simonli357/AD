import threading
import struct
import queue
import numpy as np
import cv2

from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QByteArray


class UdpConnection:
    def __init__(self, udp_socket=None, server=None):
        self.socket = udp_socket
        self.server = server
        self.MAX_DGRAM = 65507

        self._raw_image = queue.Queue(maxsize=1)
        self._raw_depth = queue.Queue(maxsize=1)
        self._raw_other = queue.Queue(maxsize=1)

        self.rgb_buf = queue.Queue(maxsize=1)
        self.depth_buf = queue.Queue(maxsize=1)
        self.depth_arr_buf = queue.Queue(maxsize=1)
        self.lane2_buf = queue.Queue(maxsize=1)
        self.road_object_buf = queue.Queue(maxsize=1)
        self.waypoint_buf = queue.Queue(maxsize=1)
        self.sign_buf = queue.Queue(maxsize=1)
        self.steer_buf = queue.Queue(maxsize=1)
        self.sw_load_buf = queue.Queue(maxsize=1)

        if udp_socket is not None:
            threading.Thread(target=self._receive_loop, daemon=True).start()
            threading.Thread(target=self._image_worker, daemon=True).start()
            threading.Thread(target=self._depth_worker, daemon=True).start()
            threading.Thread(target=self._other_worker, daemon=True).start()

    def broadcast(self, payload):
        for key in self.server.dashboard_clients.keys():
            self.socket.sendto(payload, (key, self.server.udp_port))

    def _receive_loop(self):
        """Read UDP datagrams and push raw payloads into one of three queues."""
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 5:
                    continue
                if self.server.is_host:
                    self.broadcast(seg)
                typ = seg[4]
                payload = seg[5:]
                if typ == 5:       # RGB frame
                    self._enqueue_raw(self._raw_image, payload)
                elif typ == 6:     # Depth frame
                    self._enqueue_raw(self._raw_depth, payload)
                else:              # everything else
                    if typ in (1, 2, 3, 4, 7, 8):
                        self._enqueue_raw(self._raw_other, (typ, payload))
            except Exception:
                continue

    def _enqueue_raw(self, q: queue.Queue, item):
        """Try to put item into q, dropping it if q is full."""
        try:
            q.put_nowait(item)
        except queue.Full:
            pass

    def _image_worker(self):
        while True:
            raw = self._raw_image.get()
            try:
                pix = QPixmap()
                pix.loadFromData(QByteArray(raw))
                self._try_put(self.rgb_buf, pix)
            except Exception as e:
                print(e)

    def _depth_worker(self):
        while True:
            raw = self._raw_depth.get()
            try:
                np_array = np.frombuffer(raw, dtype=np.uint8)
                depth = cv2.imdecode(np_array, cv2.IMREAD_UNCHANGED)
                depth_normalized = cv2.normalize(depth, None, 50, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_TURBO)  # TURBO colormap for better contrast
                h, w, ch = depth_colored.shape
                bytes_per_line = ch * w
                qt_image = QImage(depth_colored.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pix = QPixmap.fromImage(qt_image)
                self._try_put(self.depth_buf, pix)
                self._try_put(self.depth_arr_buf, depth)
            except Exception as e:
                print("depth_worker error:", e)
                continue

    def _other_worker(self):
        while True:
            typ, raw = self._raw_other.get()
            try:
                if typ == 1:  # lane2
                    msg = Lane2Msg().decode(raw)
                    self._try_put(self.lane2_buf, msg)
                elif typ == 2:  # road object
                    msg = Float32MultiArray().deserialize(raw)
                    self._try_put(self.road_object_buf, msg)
                elif typ == 3:  # waypoint
                    msg = Float32MultiArray().deserialize(raw)
                    self._try_put(self.waypoint_buf, msg)
                elif typ == 4:  # sign
                    msg = Float32MultiArray().deserialize(raw)
                    self._try_put(self.sign_buf, msg)
                elif typ == 7:  # steer
                    val = struct.unpack('f', raw[:4])[0]
                    self._try_put(self.steer_buf, val)
                elif typ == 8:  # sw_load
                    msg = SWLoadMsg().decode(raw)
                    self._try_put(self.sw_load_buf, msg)
            except Exception:
                pass

    def _try_put(self, buf: queue.Queue, item):
        """Helper: put item into buf if empty; if full, drop it."""
        try:
            buf.put_nowait(item)
        except queue.Full:
            pass

    def parse_rgb_image(self):
        try:
            return self.rgb_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_depth_image(self):
        try:
            return self.depth_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_depth_arr(self):
        try:
            return self.depth_arr_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_lane2(self):
        try:
            return self.lane2_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_road_object(self):
        try:
            return self.road_object_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_waypoint(self):
        try:
            return self.waypoint_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_sign(self):
        try:
            return self.sign_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_steer(self):
        try:
            return self.steer_buf.get_nowait()
        except queue.Empty:
            return None

    def parse_sw_load(self):
        try:
            return self.sw_load_buf.get_nowait()
        except queue.Empty:
            return None
