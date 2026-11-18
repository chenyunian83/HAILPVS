"""在静态扫掠总览上回放本次仿真保存的车辆姿态。"""

import math
import time
from bisect import bisect_left
from itertools import accumulate

from PyQt5.QtCore import QPoint, QSignalBlocker, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (QComboBox, QDialog, QDoubleSpinBox, QHBoxLayout,
                             QLabel, QPushButton, QSlider, QStyle, QStyleOptionSlider,
                             QVBoxLayout)
from shapely.geometry import Point

from config import SIM_TURN_SPEED

from app.rendering.sweep_view import build_playback_figure, draw_overview_pose, right_angle_invasion
from .qtutil import _primary_screen_geometry


class PlaybackSlider(QSlider):
    markerClicked = pyqtSignal(str)

    def __init__(self, markers):
        super().__init__(Qt.Horizontal)
        self.markers = markers
        self.setMinimumHeight(30)

    def _marker_positions(self):
        option = QStyleOptionSlider()
        self.initStyleOption(option)
        for key, value, color, top in self.markers:
            option.sliderPosition = option.sliderValue = value
            handle = self.style().subControlRect(QStyle.CC_Slider, option, QStyle.SC_SliderHandle, self)
            y = handle.top() - 4 if top else handle.bottom() + 4
            yield key, QPoint(handle.center().x(), y), color

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        for _, position, color in self._marker_positions():
            painter.setPen(QPen(QColor(color), 5, Qt.SolidLine, Qt.RoundCap))
            painter.drawPoint(position)
        painter.end()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            for key, position, _ in self._marker_positions():
                if (event.pos() - position).manhattanLength() <= 4:
                    self.markerClicked.emit(key)
                    event.accept()
                    return
        super().mousePressEvent(event)


class SweepPlaybackDialog(QDialog):
    _SLIDER_MAX = 10000
    _TURN_COLOR = "#2b639d"

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
        self.clearance = result.playback_clearance
        self.clearance_color = "#ad332d" if self.clearance and self.clearance.meters < 0 else "#87570f"
        self.events = dict(zip(("entry", "exit"), result.playback_turn_frames))
        if self.clearance is not None:
            self.events["minimum"] = self.clearance.frame
        contact = right_angle_invasion(result)
        if contact is not None:
            body, _, location, _ = contact
            point = Point(location)
            self.events["invasion"] = min(range(len(self.frames)),
                                          key=lambda i: self._invasion_distance(i, body, point))
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._advance)
        figure, self.axes = build_playback_figure(result)
        self.canvas = FigureCanvasQTAgg(figure)
        self.canvas.setMinimumSize(640, 400)
        self._build_controls()
        self._show_frame(0)

    def _build_controls(self):
        layout = QVBoxLayout(self)
        layout.addWidget(self.canvas, 1)
        markers = [(key, self._progress_value(self.distances[index]),
                    self.clearance_color if key == "minimum" else self._TURN_COLOR, key == "minimum")
                   for key, index in self.events.items() if key != "invasion"]
        self.progress = PlaybackSlider(markers)
        self.progress.setAccessibleName("行驶距离进度")
        self.progress.setRange(0, self._SLIDER_MAX)
        self.progress.setEnabled(self.length > 0)
        self.progress.valueChanged.connect(self._seek_distance)
        self.progress.markerClicked.connect(self.seek_event)
        layout.addWidget(self.progress)

        event_row = QHBoxLayout()
        for key, title, detail in (("entry", "入弯", "仿真参考点开始转向"),
                                   ("minimum", "最小间距", "全程最小间距所在姿态"),
                                   ("exit", "出弯", "仿真参考点结束转向")):
            if key in self.events:
                color = self.clearance_color if key == "minimum" else self._TURN_COLOR
                button = QPushButton(title)
                button.setObjectName(f"Replay-{key}")
                button.setStyleSheet(f"color: {color};")
                button.setToolTip(f"{detail} · 行驶距离 {self.distances[self.events[key]]:.2f} m")
                button.clicked.connect(lambda checked=False, target=key: self.seek_event(target))
                event_row.addWidget(button)
        event_row.addStretch()
        if self.clearance is not None:
            readout = QLabel(f"全程最小间距 {self.clearance.meters:.3f} m · {self.clearance.body}")
            readout.setStyleSheet(f"color: {self.clearance_color};")
            readout.setToolTip("逐帧量测车货外廓到路缘的水平间距，负值表示越界")
            event_row.addWidget(readout)
        layout.addLayout(event_row)

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

        if "invasion" in self.events:
            locate = QPushButton("定位侵入")
            locate.setToolTip("跳到离标记点最近的仿真姿态")
            locate.clicked.connect(lambda: self.seek_event("invasion"))
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

    def _progress_value(self, distance):
        return round(distance / self.length * self._SLIDER_MAX) if self.length else 0

    def _show_frame(self, index):
        if index == self.index:
            return
        self.index = index
        for artist in self.pose_artists:
            artist.remove()
        self.pose_artists = draw_overview_pose(self.axes, self.frames[index], self.result.hydraulic)
        if self.clearance is not None and index == self.clearance.frame:
            points = (self.clearance.body_point, self.clearance.road_point)
            self.pose_artists += self.axes.plot(*zip(*points), color=self.clearance_color,
                                                marker="o", mfc="white", ms=5, lw=2.0, zorder=13)
        distance = self.distances[index]
        with QSignalBlocker(self.progress), QSignalBlocker(self.seek_time):
            if not self.progress.isSliderDown():
                self.progress.setValue(self._progress_value(distance))
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

    def seek_event(self, key):
        """暂停并跳到原始采样帧，避免滑块舍入选到相邻姿态。"""
        if key in self.events:
            self._pause()
            self._show_frame(self.events[key])

    def _invasion_distance(self, index, body, point):
        # 侵入点来自连续扫掠面，可能在两帧之间。
        _, _, envelope, deck, cargo = self.frames[index][:5]
        if body == "cargo":
            return cargo.distance(point)
        if envelope.geom_type == "MultiPolygon":
            return min(deck.distance(point), envelope.geoms[-1].distance(point))
        return deck.distance(point)

    def done(self, result):
        self.timer.stop()
        super().done(result)
