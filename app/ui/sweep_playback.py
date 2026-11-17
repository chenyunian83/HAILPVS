"""在静态扫掠总览上回放本次仿真保存的车辆姿态。"""

import math
import time
from bisect import bisect_left
from itertools import accumulate

from PyQt5.QtCore import QSignalBlocker, Qt, QTimer
from PyQt5.QtWidgets import (QComboBox, QDialog, QDoubleSpinBox, QHBoxLayout,
                             QLabel, QPushButton, QSlider, QVBoxLayout)
from shapely.geometry import Point

from config import SIM_TURN_SPEED

from app.rendering.sweep_view import build_playback_figure, draw_overview_pose, right_angle_invasion
from .qtutil import _primary_screen_geometry


class SweepPlaybackDialog(QDialog):
    _SLIDER_MAX = 10000

    def __init__(self, result, parent=None):
        super().__init__(parent)
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

        self.setWindowTitle("扫掠回放")
        geo = _primary_screen_geometry()
        self.resize(min(1260, int(geo.width() * 0.92)), min(940, int(geo.height() * 0.90)))
        self.result = result
        self.frames = result.playback_frames
        steps = (math.dist(a[:2], b[:2]) for a, b in zip(self.frames, self.frames[1:]))
        self.distances = list(accumulate(steps, initial=0.0))
        self.length = self.distances[-1]
        self.duration = self.length / SIM_TURN_SPEED
        self.index = -1
        self.pose_artists = []
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._advance)
        figure, self.axes = build_playback_figure(result)
        self.canvas = FigureCanvasQTAgg(figure)
        self._build_controls()
        self._show_frame(0)

    def _build_controls(self):
        layout = QVBoxLayout(self)
        layout.addWidget(self.canvas, 1)
        self.progress = QSlider(Qt.Horizontal)
        self.progress.setAccessibleName("行驶距离进度")
        self.progress.setRange(0, self._SLIDER_MAX)
        self.progress.setEnabled(self.length > 0)
        self.progress.valueChanged.connect(self._seek_distance)
        layout.addWidget(self.progress)

        row = QHBoxLayout()
        self.play = QPushButton("播放")
        self.play.setEnabled(self.duration > 0)
        self.play.clicked.connect(self._toggle_play)
        self.previous = QPushButton("上一帧")
        self.previous.clicked.connect(lambda: self._step(-1))
        self.next = QPushButton("下一帧")
        self.next.clicked.connect(lambda: self._step(1))
        row.addWidget(self.play)
        row.addWidget(self.previous)
        row.addWidget(self.next)

        contact = right_angle_invasion(self.result)
        if contact is not None:
            locate = QPushButton("定位侵入")
            locate.setToolTip("跳到离标记点最近的仿真姿态")
            locate.clicked.connect(self._seek_invasion)
            row.addWidget(locate)
        self.distance_label = QLabel()
        row.addWidget(self.distance_label, 1, Qt.AlignCenter)
        row.addWidget(QLabel("仿真时间"))
        self.seek_time = QDoubleSpinBox()
        self.seek_time.setAccessibleName("仿真时间")
        self.seek_time.setRange(0, self.duration)
        self.seek_time.setSuffix(" s")
        self.seek_time.setKeyboardTracking(False)
        self.seek_time.lineEdit().textEdited.connect(self._pause)
        self.seek_time.valueChanged.connect(self._seek_seconds)
        row.addWidget(self.seek_time)
        row.addWidget(QLabel(f"/ {self.duration:.2f} s"))
        self.rate = QComboBox()
        self.rate.setAccessibleName("播放倍速")
        for speed in (0.25, 0.5, 1.0, 2.0, 4.0):
            self.rate.addItem(f"{speed:g}×", speed)
        self.rate.setCurrentIndex(2)
        self.rate.currentIndexChanged.connect(self._rebase_clock)
        row.addWidget(self.rate)
        layout.addLayout(row)
        for button in self.findChildren(QPushButton):
            button.setAutoDefault(False)

    def _show_frame(self, index):
        if index == self.index:
            return
        self.index = index
        for artist in self.pose_artists:
            artist.remove()
        self.pose_artists = draw_overview_pose(self.axes, self.frames[index], self.result.hydraulic)
        distance = self.distances[index]
        with QSignalBlocker(self.progress), QSignalBlocker(self.seek_time):
            if not self.progress.isSliderDown():
                self.progress.setValue(round(distance / self.length * self._SLIDER_MAX) if self.length else 0)
            self.seek_time.setValue(distance / SIM_TURN_SPEED)
        self.distance_label.setText(f"{distance:.2f} / {self.length:.2f} m")
        self.previous.setEnabled(index > 0)
        self.next.setEnabled(index < len(self.frames) - 1)
        self.canvas.draw_idle()

    def _show_distance(self, distance):
        if distance <= 0:
            self._show_frame(0)
        elif distance >= self.length:
            self._show_frame(len(self.frames) - 1)
        else:
            after = bisect_left(self.distances, distance)
            before = after - 1
            closest = before if distance - self.distances[before] < self.distances[after] - distance else after
            self._show_frame(closest)

    def _pause(self):
        self.timer.stop()
        self.play.setText("播放")

    def _seek_distance(self, value):
        self._pause()
        self._show_distance(value / self._SLIDER_MAX * self.length)

    def _seek_seconds(self, seconds):
        self._pause()
        destination = self.length if seconds == self.seek_time.maximum() else seconds * SIM_TURN_SPEED
        self._show_distance(destination)

    def _step(self, delta):
        self._pause()
        self._show_frame(max(0, min(len(self.frames) - 1, self.index + delta)))

    def _rebase_clock(self):
        self._clock_origin = time.monotonic()
        self._distance_origin = self.distances[self.index]

    def _toggle_play(self):
        if self.timer.isActive():
            self._pause()
            return
        if self.index == len(self.frames) - 1:
            self._show_frame(0)
        self._rebase_clock()
        self.play.setText("暂停")
        self.timer.start()

    def _advance(self):
        elapsed = time.monotonic() - self._clock_origin
        distance = self._distance_origin + elapsed * self.rate.currentData() * SIM_TURN_SPEED
        self._show_distance(distance)
        if distance >= self.length:
            self._pause()

    def _seek_invasion(self):
        self._pause()
        body, _, location, _ = right_angle_invasion(self.result)
        point = Point(location)
        # 侵入点取自连续扫掠面。有些点位于两帧之间，因此按最近的原始姿态定位。
        index = min(range(len(self.frames)), key=lambda i: self._invasion_distance(i, body, point))
        self._show_frame(index)

    def _invasion_distance(self, index, body, point):
        _, _, envelope, deck, cargo = self.frames[index][:5]
        if body == "cargo":
            return cargo.distance(point)
        if envelope.geom_type == "MultiPolygon":
            return min(deck.distance(point), envelope.geoms[-1].distance(point))
        return deck.distance(point)

    def done(self, result):
        self.timer.stop()
        super().done(result)
