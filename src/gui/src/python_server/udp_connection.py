import threading
import struct
import queue

from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QByteArray


class UdpConnection:
    def __init__(self, udp_socket=None):
        self.socket = udp_socket
        self.MAX_DGRAM = 65507

        self._raw = {
            'lane2': queue.Queue(maxsize=1),
            'road': queue.Queue(maxsize=1),
            'waypoint': queue.Queue(maxsize=1),
            'sign': queue.Queue(maxsize=1),
            'rgb': queue.Queue(maxsize=1),
            'depth': queue.Queue(maxsize=1),
            'steer': queue.Queue(maxsize=1),
            'sw_load': queue.Queue(maxsize=1),
        }

        self.lane2_buf = queue.Queue(maxsize=1)
        self.road_object_buf = queue.Queue(maxsize=1)
        self.waypoint_buf = queue.Queue(maxsize=1)
        self.sign_buf = queue.Queue(maxsize=1)
        self.rgb_buf = queue.Queue(maxsize=1)
        self.depth_buf = queue.Queue(maxsize=1)
        self.steer_buf = queue.Queue(maxsize=1)
        self.sw_load_buf = queue.Queue(maxsize=1)

        if udp_socket is not None:
            threading.Thread(target=self._receive_loop, daemon=True).start()
            threading.Thread(target=self._steer_worker, daemon=True).start()
            threading.Thread(target=self._lane2_worker, daemon=True).start()
            threading.Thread(target=self._road_worker, daemon=True).start()
            threading.Thread(target=self._waypoint_worker, daemon=True).start()
            threading.Thread(target=self._sign_worker, daemon=True).start()
            threading.Thread(target=self._image_worker, daemon=True).start()
            threading.Thread(target=self._depth_worker, daemon=True).start()
            threading.Thread(target=self._sw_load_worker, daemon=True).start()

    def _receive_loop(self):
        """Demultiplex incoming UDP packets into type‐specific raw queues."""
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 6:
                    continue
                typ = seg[4]
                payload = seg[5:]
                if typ == 1:
                    self._enqueue_raw('lane2', payload)
                elif typ == 2:
                    self._enqueue_raw('road', payload)
                elif typ == 3:
                    self._enqueue_raw('waypoint', payload)
                elif typ == 4:
                    self._enqueue_raw('sign', payload)
                elif typ == 5:
                    self._enqueue_raw('rgb', payload)
                elif typ == 6:
                    self._enqueue_raw('depth', payload)
                elif typ == 7:
                    self._enqueue_raw('steer', payload)
                elif typ == 8:
                    self._enqueue_raw('sw_load', payload)
            except Exception:
                # ignore transient errors
                continue

    def _enqueue_raw(self, key, raw):
        """Try to append raw bytes into its queue, dropping old if necessary."""
        q = self._raw[key]
        try:
            q.put_nowait(raw)
        except queue.Full:
            pass

    def _steer_worker(self):
        while True:
            raw = self._raw['steer'].get()
            try:
                val = struct.unpack('f', raw[:4])[0]
                self._try_put(self.steer_buf, val)
            except Exception:
                pass

    def _lane2_worker(self):
        while True:
            raw = self._raw['lane2'].get()
            try:
                msg = Lane2Msg().decode(raw)
                self._try_put(self.lane2_buf, msg)
            except Exception:
                pass

    def _road_worker(self):
        while True:
            raw = self._raw['road'].get()
            try:
                msg = Float32MultiArray().deserialize(raw)
                self._try_put(self.road_object_buf, msg)
            except Exception:
                pass

    def _waypoint_worker(self):
        while True:
            raw = self._raw['waypoint'].get()
            try:
                msg = Float32MultiArray().deserialize(raw)
                self._try_put(self.waypoint_buf, msg)
            except Exception:
                pass

    def _sign_worker(self):
        while True:
            raw = self._raw['sign'].get()
            try:
                msg = Float32MultiArray().deserialize(raw)
                self._try_put(self.sign_buf, msg)
            except Exception:
                pass

    def _image_worker(self):
        while True:
            raw = self._raw['rgb'].get()
            try:
                pix = QPixmap()
                pix.loadFromData(QByteArray(raw))
                self._try_put(self.rgb_buf, pix)
            except Exception:
                pass

    def _depth_worker(self):
        while True:
            raw = self._raw['depth'].get()
            try:
                pix = QPixmap()
                pix.loadFromData(QByteArray(raw))
                self._try_put(self.depth_buf, pix)
            except Exception:
                pass

    def _sw_load_worker(self):
        while True:
            raw = self._raw['sw_load'].get()
            try:
                msg = SWLoadMsg().decode(raw)
                self._try_put(self.sw_load_buf, msg)
            except Exception:
                pass

    def _try_put(self, buf, item):
        """Helper: append to buf if empty, else drop."""
        try:
            buf.put_nowait(item)
        except queue.Full:
            pass

    def parse_steer(self):
        try:
            return self.steer_buf.get_nowait()
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

    def parse_sw_load(self):
        try:
            return self.sw_load_buf.get_nowait()
        except queue.Empty:
            return None
