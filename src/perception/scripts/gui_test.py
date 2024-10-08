import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

class OpenCVGuiApp(QWidget):
    def __init__(self):
        super().__init__()

        # Set up the window
        self.setWindowTitle('PyQt5 and OpenCV GUI')
        self.setGeometry(100, 100, 800, 600)

        # Set up layout
        self.layout = QVBoxLayout()

        # OpenCV Video Display
        self.video_label = QLabel(self)
        self.layout.addWidget(self.video_label)

        # Set up buttons
        self.button_layout = QHBoxLayout()
        self.start_button = QPushButton('Start')
        self.stop_button = QPushButton('Stop')
        self.pause_button = QPushButton('Pause')
        self.resume_button = QPushButton('Resume')

        self.button_layout.addWidget(self.start_button)
        self.button_layout.addWidget(self.stop_button)
        self.button_layout.addWidget(self.pause_button)
        self.button_layout.addWidget(self.resume_button)

        self.layout.addLayout(self.button_layout)

        # Set up sliders
        self.slider1 = QSlider(Qt.Horizontal)
        self.slider1.setRange(0, 100)
        self.slider1.setValue(50)
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider2.setRange(0, 100)
        self.slider2.setValue(50)

        self.layout.addWidget(self.slider1)
        self.layout.addWidget(self.slider2)

        # Set the main layout
        self.setLayout(self.layout)

        # Connect buttons to functions
        self.start_button.clicked.connect(self.start_video)
        self.stop_button.clicked.connect(self.stop_video)
        self.pause_button.clicked.connect(self.pause_video)
        self.resume_button.clicked.connect(self.resume_video)

        # Video capture and timer
        self.cap = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.paused = False

    def start_video(self):
        # Start capturing video if not already started
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)  # 0 for default camera
            if not self.cap.isOpened():
                print("Error: Could not open camera.")
                return
        self.timer.start(30)  # Set timer to update every 30 ms
        self.paused = False

    def stop_video(self):
        # Stop the video
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.video_label.clear()

    def pause_video(self):
        # Pause the video
        self.paused = True

    def resume_video(self):
        # Resume the video
        self.paused = False

    def update_frame(self):
        if self.cap is not None and not self.paused:
            ret, frame = self.cap.read()
            if ret:
                # Apply slider effects (for demonstration)
                brightness = self.slider1.value()
                contrast = self.slider2.value()
                frame = cv2.convertScaleAbs(frame, alpha=contrast/50, beta=brightness-50)

                # Convert the frame to RGB format
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channel = frame.shape
                step = channel * width
                q_img = QImage(frame.data, width, height, step, QImage.Format_RGB888)
                self.video_label.setPixmap(QPixmap.fromImage(q_img))

    def closeEvent(self, event):
        # Release the video capture when the window is closed
        if self.cap is not None:
            self.cap.release()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = OpenCVGuiApp()
    window.show()
    sys.exit(app.exec_())