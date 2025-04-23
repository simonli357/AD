import threading
import queue
import numpy as np
import cv2
import struct

from std_msgs.msg import Float32MultiArray
from python_server.msg.lane2_msg import Lane2Msg
from python_server.msg.sw_load_msg import SWLoadMsg
from PyQt5.QtGui import QPixmap, QImage


class UdpConnection:
    def __init__(self,
                 udp_socket,
                 on_rgb_frame=None,
                 on_depth_frame=None,
                 on_lane2=None,
                 on_road_obj=None,
                 on_waypoint=None,
                 on_sign=None,
                 on_steer=None,
                 on_sw_load=None):
        self.socket = udp_socket
        self.MAX_DGRAM = 65507

        # callbacks (Qt signal .emit methods)
        self.on_rgb_frame = on_rgb_frame
        self.on_depth_frame = on_depth_frame
        self.on_lane2 = on_lane2
        self.on_road_obj = on_road_obj
        self.on_waypoint = on_waypoint
        self.on_sign = on_sign
        self.on_steer = on_steer
        self.on_sw_load = on_sw_load

        # internal queues for each message type
        self._queues = {
            'rgb': queue.Queue(maxsize=1),
            'depth': queue.Queue(maxsize=1),
            'lane2': queue.Queue(maxsize=1),
            'road': queue.Queue(maxsize=1),
            'wayp': queue.Queue(maxsize=1),
            'sign': queue.Queue(maxsize=1),
            'steer': queue.Queue(maxsize=1),
            'sw': queue.Queue(maxsize=1),
        }

        # dispatch table: raw handlers enqueue
        self.data_actions = {
            1: lambda b: self._enqueue('lane2', b),
            2: lambda b: self._enqueue('road', b),
            3: lambda b: self._enqueue('wayp', b),
            4: lambda b: self._enqueue('sign', b),
            5: lambda b: self._enqueue('rgb', b),
            6: lambda b: self._enqueue('depth', b),
            7: lambda b: self._enqueue('steer', b),
            8: lambda b: self._enqueue('sw', b),
        }

        # start receive thread
        threading.Thread(target=self._receive_loop, daemon=True).start()
        # start worker threads
        threading.Thread(target=self._worker_rgb, daemon=True).start()
        threading.Thread(target=self._worker_depth, daemon=True).start()
        threading.Thread(target=self._worker_lane2, daemon=True).start()
        threading.Thread(target=self._worker_road, daemon=True).start()
        threading.Thread(target=self._worker_wayp, daemon=True).start()
        threading.Thread(target=self._worker_sign, daemon=True).start()
        threading.Thread(target=self._worker_steer, daemon=True).start()
        threading.Thread(target=self._worker_sw, daemon=True).start()

    def _receive_loop(self):
        while True:
            try:
                seg, _ = self.socket.recvfrom(self.MAX_DGRAM)
                if len(seg) < 5:
                    continue
                msg_type = seg[4]
                payload = seg[5:]
                handler = self.data_actions.get(msg_type)
                if handler:
                    handler(payload)
            except Exception:
                # ignore invalid or closed socket
                pass

    def _enqueue(self, key, raw_bytes):
        q = self._queues[key]
        try:
            q.put_nowait(raw_bytes)
        except Exception:
            pass

    # worker threads:
    def _worker_rgb(self):
        while True:
            raw = self._queues['rgb'].get()
            arr = np.frombuffer(raw, dtype=np.uint8)
            cv_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg)
            if self.on_rgb_frame:
                self.on_rgb_frame(pix)

    def _worker_depth(self):
        while True:
            raw = self._queues['depth'].get()
            arr = np.frombuffer(raw, dtype=np.uint8)
            cv_img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED).astype(np.uint16)
            norm = cv2.normalize(cv_img, None, 50, 255, cv2.NORM_MINMAX)
            col = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_TURBO)
            h, w, ch = col.shape
            qimg = QImage(col.data, w, h, ch * w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg)
            if self.on_depth_frame:
                self.on_depth_frame(pix)

    def _worker_lane2(self):
        while True:
            raw = self._queues['lane2'].get()
            msg = Lane2Msg().decode(raw)
            if self.on_lane2:
                self.on_lane2(msg)

    def _worker_road(self):
        while True:
            raw = self._queues['road'].get()
            msg = Float32MultiArray().deserialize(raw)
            if self.on_road_obj:
                self.on_road_obj(msg)

    def _worker_wayp(self):
        while True:
            raw = self._queues['wayp'].get()
            msg = Float32MultiArray().deserialize(raw)
            if self.on_waypoint:
                self.on_waypoint(msg)

    def _worker_sign(self):
        while True:
            raw = self._queues['sign'].get()
            msg = Float32MultiArray().deserialize(raw)
            if self.on_sign:
                self.on_sign(msg)

    def _worker_steer(self):
        while True:
            raw = self._queues['steer'].get()
            val = struct.unpack('f', raw[:4])[0]
            if self.on_steer:
                self.on_steer(val)

    def _worker_sw(self):
        while True:
            raw = self._queues['sw'].get()
            msg = SWLoadMsg().decode(raw)
            if self.on_sw_load:
                self.on_sw_load(msg)
