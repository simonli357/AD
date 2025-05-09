import threading
import struct
import queue
import numpy as np
import cv2

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QByteArray
from collections import OrderedDict


class UdpConnection:
    def __init__(self, udp_socket=None, server=None):
        self.socket = udp_socket
        self.server = server
        self.MAX_DGRAM = 65507

        self._raw_image = queue.Queue(maxsize=1024)
        self._raw_depth = queue.Queue(maxsize=1024)
        self._raw_other = queue.Queue()

        self.image_map = OrderedDict()
        self.depth_map = OrderedDict()

        self.buf_size = 2097152
        self.socket.setsockopt(udp_socket.SOL_SOCKET, udp_socket.SO_RCVBUF, self.buf_size)
        self.socket.setsockopt(udp_socket.SOL_SOCKET, udp_socket.SO_SNDBUF, self.buf_size)

        self.rgb_buf = queue.Queue(maxsize=1)
        self.depth_buf = queue.Queue(maxsize=1)
        self.depth_arr_buf = queue.Queue(maxsize=1)
        self.lane2_buf = queue.Queue(maxsize=1)
        self.road_object_buf = queue.Queue(maxsize=1)
        self.waypoint_buf = queue.Queue(maxsize=1)
        self.sign_buf = queue.Queue(maxsize=1)
        self.steer_buf = queue.Queue(maxsize=1)
        self.sw_load_buf = queue.Queue(maxsize=1)
        self.states_buf = queue.Queue(maxsize=1)

        self.show_depth = False
        self.alive = True

        if udp_socket is not None:
            threading.Thread(target=self._receive_loop, daemon=True).start()
            threading.Thread(target=self._other_worker, daemon=True).start()
            threading.Thread(target=self._image_worker, daemon=True).start()
            threading.Thread(target=self._depth_worker, daemon=True).start()

    def broadcast(self, payload):
        for key in self.server.dashboard_clients.keys():
            self.socket.sendto(payload, (key, self.server.udp_port))

    def _receive_loop(self):
        """Read UDP datagrams and push raw payloads into one of three queues."""
        while self.alive:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 5:
                    continue
                if self.server.is_host:
                    self.broadcast(seg)
                typ = seg[4]
                payload = seg[5:]
                if typ == 5:       # RGB frame
                    num_segments = struct.unpack('<H', seg[:2])[0]
                    seg_num = struct.unpack('<H', seg[2:4])[0]
                    self._enqueue_raw(self._raw_image, payload)
                elif typ == 6:     # Depth frame
                    num_segments = struct.unpack('<H', seg[:2])[0]
                    seg_num = struct.unpack('<H', seg[2:4])[0]
                    self._enqueue_raw(self._raw_depth, (num_segments, seg_num, payload))
                else:              # everything else
                    if typ in (1, 2, 3, 4, 7, 8, 9):
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
        while self.alive:
            try:
                raw = self._raw_image.get()
                pix = QPixmap()
                pix.loadFromData(QByteArray(raw))
                self._try_put(self.rgb_buf, pix)
            except Exception:
                continue

    def _depth_worker(self):
        while self.alive:
            try:
                num_segments, seg_num, payload = self._raw_depth.get()
                self.depth_map[seg_num] = payload
                if len(self.depth_map.keys()) == num_segments:
                    raw = b''.join(self.depth_map.values())
                    self.depth_map.clear()
                else:
                    continue
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
            except Exception:
                continue

    def _other_worker(self):
        while self.alive:
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
                elif typ == 9:  # states
                    msg = Pose().deserialize(raw)
                    self._try_put(self.states_buf, msg)
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

    def parse_states(self):
        try:
            return self.states_buf.get_nowait()
        except queue.Empty:
            return None
