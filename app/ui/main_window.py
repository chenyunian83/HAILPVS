"""主工作台：并排展示工况、扫掠图及逐项计算过程。"""
import datetime
import os
from io import BytesIO
from pathlib import Path

from PyQt5.QtCore import QSettings, QSignalBlocker, Qt, QUrl
from PyQt5.QtGui import QKeySequence, QPixmap, QTextOption
from PyQt5.QtWidgets import (
    QActionGroup,
    QApplication,
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from config import (
    ACCEPTANCE_NOTE,
    APP_NAME,
    APP_VERSION,
    EXPORT_PWD,
    HIGHWAY_GRADES,
    HYDRAULIC_AXLE_DEFAULT_STEER_DEG,
    MODEL_SUFFIX,
    ROAD_TYPES,
    TRACTOR_SWING_EXIT_FACTOR,
    TRAILER_TYPES,
    VERT_SCENARIO,
)

try:
    from config import PAD_ITEMS
except ImportError:
    PAD_ITEMS = ["干燥的木板", "干燥的橡胶垫", "潮湿的木板", "潮湿的橡胶垫"]

from app.core.datamodel import (
    ArcCorner,
    CargoParams,
    InputBundle,
    RightAngleCorner,
    RoadParams,
    TractorParams,
    TrailerParams,
)
from app.core.passability import run_scene
from app.model.project import ensure_suffix, load_model, save_model
from app.rendering.sweep_view import PLOT_STYLES, draw_sweep_view, right_angle_invasion
from app.rendering.verdict_style import colorize_verdict_html

from .qtutil import _app_font, _primary_screen_geometry
from .sweep_playback import SweepPlaybackDialog
from .widgets import (
    ClickableImageLabel,
    ImageViewerDialog,
    Sweep3DDialog,
    _make_form_group,
    _new_combo,
    _new_edit,
    _row_label,
)
from .workspace_style import workspace_stylesheet

_CARGO_FIELDS = [
    ("货物长度 (m)", "c_length", ""),
    ("货物宽度 (m)", "c_width", ""),
    ("货物高度 (m)", "c_height", ""),
]
_TRAILER_FIELDS = [
    ("挂车长度 (m)", "t_length", ""),
    ("挂车宽度 (m)", "t_width", ""),
    ("车板离地高 (m)", "t_deck_height", ""),
    ("挂车轮距 (m)", "t_track_width", ""),
    ("牵引销至轴组中心 (m)", "t_kingpin_to_axle", ""),
    ("轴线数", "t_axle_count", ""),
    ("相邻轴线间距 (m)", "t_axle_spacing", ""),
    ("每轴最大转向角 (°)", "t_axle_steer_deg", str(HYDRAULIC_AXLE_DEFAULT_STEER_DEG)),
]
_TRACTOR_FIELDS = [
    ("牵引车轴距 (m)", "tr_wheelbase", ""),
    ("最大转向角 (°)", "tr_max_steering_angle_deg", ""),
    ("牵引车宽度 (m)", "tr_width", ""),
    ("前轮轮距 (m)", "tr_track_width", ""),
    ("鞍座至后轴距离 (m)", "tr_fifth_wheel_pos", "0.0"),
]
_ARC_FIELDS = [
    ("道路外半径 (m)", "outer_radius", ""),
    ("车道宽度 (m)", "lane_width", ""),
]
_RIGHT_FIELDS = [
    ("入口宽度 (m)", "entry_width", ""),
    ("出口宽度 (m)", "exit_width", ""),
    ("内角半径 (m)", "corner_radius", "0.0"),
    ("左转物理区可用宽 (m)", "W_section", ""),
    ("左转处可用宽 (m)", "W_lturn", ""),
]
_STABILITY_FIELDS = [
    ("最高行驶速度 (km/h)", "max_speed_kmh", "10.0"),
    ("弯道超高率", "superelevation", ""),
    ("车货重心离地高 (m)", "combined_cg_height", ""),
    ("货物重心离板高 (m)", "c_cg_height", ""),
]
_OPTIONAL_HINTS = {
    "t_kingpin_to_axle": "默认挂车长度",
    "tr_track_width": "默认牵引车宽",
    "superelevation": "默认 0.02",
    "combined_cg_height": "留空时估算",
    "c_cg_height": "默认 0",
    "W_section": "默认入口宽度",
    "W_lturn": "默认出口宽度",
}
_CLEARANCE_MODES = {"unknown": "情况待确认", "open": "上方开敞", "obstacle": "有顶部障碍"}
_EMPTY_PREVIEW = "运行弯道检验后，扫掠图将在此显示"
_DOCS = Path(__file__).resolve().parents[2] / "docs"
_STANDARD_URL = "https://xxgk.mot.gov.cn/jigou/glj/202310/t20231008_3924626.html"


class MainWindow(QMainWindow):
    _STATUS_STYLE = {
        "none": ("未运算", "#edf2f7", "#40596e"),
        "running": ("检验中", "#e0f1f8", "#12617e"),
        "ok": ("通过", "#e2f5ec", "#126047"),
        "ng": ("不通过", "#fce9ea", "#a12435"),
        "pending": ("待核实", "#fff3db", "#80551b"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}{APP_VERSION}")
        geo = _primary_screen_geometry()
        self.resize(min(1600, int(geo.width() * 0.94)), min(1000, int(geo.height() * 0.92)))
        self.setStyleSheet(workspace_stylesheet())
        self.setFont(_app_font(10))
        self.init_data()
        self.init_menu()
        self.init_ui()
        self._edit_defaults = {
            key: edit.text()
            for edits in (self.edits, self._arc_edits, self._right_edits)
            for key, edit in edits.items()
        }
        for edits in (self.edits, self._arc_edits, self._right_edits):
            for edit in edits.values():
                edit.textChanged.connect(self.action_clear_result)
        for combo in (self.cmb_pad, self.cmb_trailer_type, self.cmb_road, self.cmb_grade,
                      self.cmb_turn_direction, self.cmb_clearance_mode, self.cmb_clearance_structure):
            combo.currentTextChanged.connect(self.action_clear_result)
        for combo in (self.cmb_trailer_type, self.cmb_road, self.cmb_turn_direction, self.cmb_clearance_mode):
            combo.currentTextChanged.connect(self._sync_inputs)
        self.edits["combined_cg_height"].textChanged.connect(self._sync_inputs)

    # ================= 数据状态 =================
    def init_data(self):
        self.edits = {}
        self._arc_edits = {}
        self._right_edits = {}
        self._form_rows = {}
        self._methods_dialog = None
        self.cur_result = None
        self.current_model_path = ""
        self._orig_pixmap = None
        self._settings = QSettings("HAILPVS", "HAILPVS")
        self._plot_style = self._settings.value("drawing/style", "classic", type=str)
        if self._plot_style not in PLOT_STYLES:
            self._plot_style = "classic"

    # ================= 菜单 =================
    def init_menu(self):
        bar = self.menuBar()
        m_file = bar.addMenu("文件(&F)")
        m_file.addAction("新建模型", self.action_new, QKeySequence.New)
        m_file.addAction("打开模型", self.action_open, QKeySequence.Open)
        m_file.addSeparator()
        m_file.addAction("保存", self.action_save, QKeySequence.Save)
        m_file.addAction("另存为", self.action_saveas, QKeySequence.SaveAs)
        m_file.addSeparator()
        m_file.addAction("退出系统", self.close)
        m_tool = bar.addMenu("工具(&T)")
        m_tool.addAction("清空检验结果", self.action_clear_result)
        m_tool.addSeparator()
        m_tool.addAction("检验方法说明", self.action_show_acceptance)
        m_style = bar.addMenu("设置(&S)").addMenu("画图风格")
        self._style_actions = QActionGroup(self)
        self._style_actions.setExclusive(True)
        for style, label in PLOT_STYLES.items():
            action = m_style.addAction(label)
            action.setCheckable(True)
            action.setData(style)
            self._style_actions.addAction(action)
            action.setChecked(style == self._plot_style)
        self._style_actions.triggered.connect(self._change_plot_style)

    # ================= 界面 =================
    def init_ui(self):
        central = QWidget()
        central.setObjectName("Workspace")
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(15, 10, 15, 15)
        layout.setSpacing(10)
        layout.addWidget(self._build_header())

        workspace = QSplitter(Qt.Horizontal)
        workspace.setObjectName("WorkspaceSplitter")
        workspace.setHandleWidth(9)
        workspace.setChildrenCollapsible(False)
        self.inputs_panel = self._build_inputs()
        self.graph_panel = self._build_results()
        self.process_panel = self._build_process()
        workspace.addWidget(self.inputs_panel)
        workspace.addWidget(self.graph_panel)
        workspace.addWidget(self.process_panel)
        workspace.setStretchFactor(1, 1)
        workspace.setSizes([316, max(360, self.width() - 745), 385])
        layout.addWidget(workspace, 1)
        self._sync_inputs()
        self.action_clear_result()

    @staticmethod
    def _heading(text, style="PanelTitle"):
        label = QLabel(text)
        label.setObjectName(style)
        return label

    @staticmethod
    def _panel(name, minimum_width):
        panel = QFrame()
        panel.setObjectName(name)
        panel.setMinimumWidth(minimum_width)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(13, 12, 13, 13)
        layout.setSpacing(10)
        return panel, layout

    def _build_header(self):
        header = QFrame()
        header.setObjectName("WorkspaceHeader")
        row = QHBoxLayout(header)
        row.setContentsMargins(15, 9, 13, 10)
        row.setSpacing(7)
        titles = QVBoxLayout()
        titles.setSpacing(2)
        titles.addWidget(self._heading("弯道通过性检验", "WorkspaceTitle"))
        self.model_label = self._heading("未命名模型", "ModelName")
        self.model_label.setTextFormat(Qt.PlainText)
        titles.addWidget(self.model_label)
        row.addLayout(titles)
        row.addStretch(1)
        for label, action in (("打开", self.action_open), ("保存", self.action_save),
                              ("方法参考", self.action_show_acceptance)):
            button = QPushButton(label)
            button.setObjectName("HeaderButton")
            button.setMinimumHeight(32)
            button.clicked.connect(action)
            row.addWidget(button)
        self.btn_export = QPushButton("导出 Word")
        self.btn_export.setObjectName("HeaderButton")
        self.btn_export.setMinimumHeight(32)
        self.btn_export.clicked.connect(self.export_word)
        row.addWidget(self.btn_export)
        return header

    def _build_inputs(self):
        panel, layout = self._panel("InputPanel", 284)
        layout.addWidget(self._heading("工况参数"))
        form = QWidget()
        form.setObjectName("ConditionForm")
        form_layout = QVBoxLayout(form)
        form_layout.setContentsMargins(0, 0, 4, 1)
        form_layout.setSpacing(10)
        self._build_road_groups(form_layout)
        self._build_vehicle_groups(form_layout)
        self._build_stability_group(form_layout)
        form_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setObjectName("FormScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(form)
        layout.addWidget(scroll, 1)
        self.btn_calc = QPushButton("运行弯道检验")
        self.btn_calc.setObjectName("RunButton")
        self.btn_calc.setMinimumHeight(40)
        self.btn_calc.clicked.connect(self.run_calc)
        layout.addWidget(self.btn_calc)
        return panel

    def _build_results(self):
        panel, layout = self._panel("Canvas", 330)
        title = QHBoxLayout()
        title.setSpacing(6)
        title.addWidget(self._heading("扫掠图"))
        self.lbl_status = QLabel()
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setMinimumWidth(61)
        title.addWidget(self.lbl_status)
        title.addStretch(1)
        self.btn_preview = QPushButton()
        self.btn_preview.clicked.connect(self.open_image_viewer)
        self.btn_save_img = QPushButton("保存 PNG")
        self.btn_save_img.clicked.connect(self.save_sweep_img)
        self._update_plot_controls()
        title.addWidget(self.btn_preview)
        title.addWidget(self.btn_save_img)
        layout.addLayout(title)
        self.scenario_label = self._heading("", "ScenarioLabel")
        self.scenario_label.setWordWrap(True)
        layout.addWidget(self.scenario_label)

        self.img_label = ClickableImageLabel()
        self.img_label.setObjectName("SweepPreview")
        self.img_label.setWordWrap(True)
        self.img_label.setMinimumHeight(160)
        self.img_label.clicked.connect(self.open_image_viewer)
        layout.addWidget(self.img_label, 1)

        metrics = QHBoxLayout()
        metrics.setSpacing(7)
        self.metric_labels = []
        for _ in range(3):
            card = QFrame()
            card.setObjectName("MetricCard")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(6, 7, 6, 8)
            card_layout.setSpacing(3)
            name = self._heading("", "MetricTitle")
            name.setWordWrap(True)
            value = self._heading("", "MetricValue")
            value.setWordWrap(True)
            card_layout.addWidget(name)
            card_layout.addWidget(value)
            self.metric_labels.append((name, value))
            metrics.addWidget(card, 1)
        layout.addLayout(metrics)
        return panel

    def _build_process(self):
        panel, layout = self._panel("ProcessPanel", 285)
        layout.addWidget(self._heading("计算过程"))
        self.process_sections = QComboBox()
        self.process_sections.setObjectName("ProcessSections")
        self.process_sections.setMinimumHeight(32)
        self.process_sections.setAccessibleName("计算过程章节")
        self.process_sections.setPlaceholderText("跳转到章节")
        self.process_sections.currentIndexChanged.connect(self._jump_to_step)
        layout.addWidget(self.process_sections)
        self.text_result = QTextBrowser()
        self.text_result.setObjectName("CalculationText")
        self.text_result.setOpenLinks(False)
        self.text_result.setOpenExternalLinks(False)
        self.text_result.anchorClicked.connect(self._open_playback_link)
        self.text_result.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        self.text_result.setPlaceholderText("运行检验后查看公式、代入值和判定。")
        self.text_result.document().setDocumentMargin(5)
        layout.addWidget(self.text_result, 1)
        return panel

    def _build_methods(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("检验方法参考")
        dialog.resize(870, 690)
        layout = QVBoxLayout(dialog)
        self.methods_document = QTextBrowser()
        self.methods_document.setOpenExternalLinks(True)
        self.methods_document.document().setDocumentMargin(24)
        self.methods_document.document().setBaseUrl(QUrl.fromLocalFile(str(_DOCS) + os.sep))
        try:
            reference = (_DOCS / "检验方法参考.md").read_text(encoding="utf-8")
        except OSError:
            reference = f"# 检验方法参考\n\n{ACCEPTANCE_NOTE}\n\n[交通运输部发布公告]({_STANDARD_URL})"
        self.methods_document.setMarkdown(reference)
        layout.addWidget(self.methods_document)
        return dialog

    # ---- 工况表单 ----
    def _add_row(self, form, key, title, widget):
        label = _row_label(title)
        label.setBuddy(widget)
        widget.setAccessibleName(title)
        self._form_rows[key] = (label, widget)
        form.addRow(label, widget)

    def _add_edits(self, form, fields, edits):
        for title, key, default in fields:
            edit = _new_edit(default)
            if not default:
                edit.setPlaceholderText(_OPTIONAL_HINTS.get(key, "必填"))
            edits[key] = edit
            self._add_row(form, key, title, edit)

    def _build_road_groups(self, layout):
        scenario, form = _make_form_group("工况类型")
        self.cmb_road = _new_combo(ROAD_TYPES, ROAD_TYPES[0])
        self._add_row(form, "road_type", "弯道类型", self.cmb_road)
        self.cmb_trailer_type = _new_combo(TRAILER_TYPES, TRAILER_TYPES[0])
        self._add_row(form, "trailer_type", "挂车类型", self.cmb_trailer_type)
        self.cmb_turn_direction = _new_combo(["右转", "左转"], "右转")
        self._add_row(form, "turn_direction", "检验方向", self.cmb_turn_direction)
        layout.addWidget(scenario)

        road, form = _make_form_group("道路与净空")
        self.cmb_grade = _new_combo(HIGHWAY_GRADES, "三级公路")
        self._add_row(form, "highway_grade", "公路等级", self.cmb_grade)
        self._add_edits(form, _ARC_FIELDS, self._arc_edits)
        self._add_edits(form, _RIGHT_FIELDS, self._right_edits)
        self.cmb_clearance_mode = _new_combo(())
        for mode, label in _CLEARANCE_MODES.items():
            self.cmb_clearance_mode.addItem(label, mode)
        self._add_row(form, "vertical_clearance_mode", "上方情况", self.cmb_clearance_mode)
        self.cmb_clearance_structure = _new_combo(list(VERT_SCENARIO))
        self.cmb_clearance_structure.setPlaceholderText("请选择")
        self.cmb_clearance_structure.setCurrentIndex(-1)
        self._add_row(form, "clearance_structure_type", "障碍类型", self.cmb_clearance_structure)
        self._add_edits(form, [("最小顶部净空 (m)", "vertical_clearance", "")], self.edits)
        self.edits["vertical_clearance"].setPlaceholderText("填实测值")
        self.edits["vertical_clearance"].setToolTip("沿通行路径，路面到最低顶部障碍物底面的最小垂直距离。")
        layout.addWidget(road)

    def _build_vehicle_groups(self, layout):
        cargo, form = _make_form_group("货物")
        self._add_edits(form, _CARGO_FIELDS, self.edits)
        layout.addWidget(cargo)

        trailer, form = _make_form_group("挂车")
        self._add_edits(form, _TRAILER_FIELDS, self.edits)
        layout.addWidget(trailer)

        self.g_tractor, form = _make_form_group("牵引车")
        self._add_edits(form, _TRACTOR_FIELDS, self.edits)
        layout.addWidget(self.g_tractor)

    def _build_stability_group(self, layout):
        stability, form = _make_form_group("横向稳定性")
        self._add_edits(form, _STABILITY_FIELDS, self.edits)
        self.cmb_pad = _new_combo(PAD_ITEMS)
        self._add_row(form, "pad", "货物衬垫", self.cmb_pad)
        layout.addWidget(stability)

    def _inactive_fields(self):
        is_arc = self.cmb_road.currentText() == "圆弧弯道"
        hydraulic = "液压" in self.cmb_trailer_type.currentText()
        inactive = set()
        if is_arc:
            inactive.update(("turn_direction", *self._right_edits))
        else:
            inactive.update(("highway_grade", "tr_track_width", *self._arc_edits))
        if self.cmb_turn_direction.currentText() != "左转":
            inactive.update(("W_section", "W_lturn"))
        if hydraulic:
            inactive.update(("t_kingpin_to_axle", "tr_wheelbase", "tr_max_steering_angle_deg",
                             "tr_track_width", "tr_fifth_wheel_pos"))
            if is_arc or TRACTOR_SWING_EXIT_FACTOR == 0:
                inactive.add("tr_width")
        else:
            inactive.update(("t_axle_count", "t_axle_spacing", "t_axle_steer_deg"))
        if self.edits["combined_cg_height"].text().strip():
            inactive.add("c_cg_height")
        if self.cmb_clearance_mode.currentData() != "obstacle":
            inactive.update(("vertical_clearance", "clearance_structure_type"))
        return inactive

    def _sync_inputs(self):
        inactive = self._inactive_fields()
        for key, (label, field) in self._form_rows.items():
            label.setVisible(key not in inactive)
            field.setVisible(key not in inactive)
        self.g_tractor.setVisible(any(key not in inactive for _, key, _ in _TRACTOR_FIELDS))

    # ================= 状态徽章 =================
    def set_status_badge(self, state):
        text, bg, fg = self._STATUS_STYLE.get(state, self._STATUS_STYLE["none"])
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(
            "QLabel {"
            f"background-color: {bg}; color: {fg}; border-radius: 7px;"
            "padding: 4px 8px; font-size: 9pt; font-weight: 700;}")

    def _refresh_metrics(self):
        road_type = self.cmb_road.currentText()
        is_right_angle = road_type == "直角弯道"
        grade = self.cmb_grade.currentText()
        road_detail = self.cmb_turn_direction.currentText() if is_right_angle else grade
        self.scenario_label.setText(f"{road_type}  /  {road_detail}  /  {self.cmb_trailer_type.currentText()}")
        top_name = "顶部净空间隙" if self.cmb_clearance_mode.currentData() == "obstacle" else "顶部净空"
        names = ("所需宽度", "最大侵入深度", top_name) if is_right_angle else (
            "转弯通道宽度", "最小稳态净空", top_name)
        values = (None, None, None)
        if self.cur_result is not None:
            metrics = self.cur_result.metrics
            if is_right_angle:
                width = metrics["required_w"]
                horizontal = None if metrics["split_error"] else max(
                    metrics["vehicle_depth"], metrics["cargo_depth"])
            else:
                width = metrics["turn_channel_w"]
                horizontal = None if metrics["sim_unstable"] else min(
                    metrics["clearance_inner"], metrics["clearance_outer"])
                if metrics["lookup"] == "pass":
                    names = ("通道宽度（参考）", *names[1:])
            vertical = metrics["vertical_clearance_gap"]
            if vertical is None:
                vertical = metrics["vertical_clearance_status"]
            values = (width, horizontal, vertical)
        for (title, output), name, value in zip(self.metric_labels, names, values):
            title.setText(name)
            if isinstance(value, str):
                output.setText(value)
            else:
                output.setText("—" if value is None else f"{value:.3f} m")

    def _set_model_path(self, path):
        self.current_model_path = path
        name = os.path.basename(path) if path else "未命名模型"
        self.model_label.setText(self.model_label.fontMetrics().elidedText(name, Qt.ElideMiddle, 290))
        self.model_label.setToolTip(path)

    # ================= 表单采集 → InputBundle =================
    def _f(self, key):
        return self._number(self.edits[key])

    def _f_arc(self, key):
        return self._number(self._arc_edits[key])

    def _f_right(self, key):
        return self._number(self._right_edits[key])

    @staticmethod
    def _number(edit, *, optional=False):
        text = edit.text().strip()
        if not text:
            if optional:
                return None
            raise ValueError(f"请填写{edit.accessibleName()}")
        try:
            return float(text)
        except ValueError:
            raise ValueError(f"{edit.accessibleName()}请输入数字") from None

    def collect_bundle(self) -> InputBundle:
        inactive = self._inactive_fields()

        def field(key, default=None, *, optional=False):
            if key in inactive:
                return default
            return self._number(self.edits[key], optional=optional)

        cargo_w = self._f("c_width")
        cargo_l = self._f("c_length")
        trailer_w = self._f("t_width")
        trailer_l = self._f("t_length")
        # 货物超出车板（宽度/长度）时按“超宽/超长装载”处理，允许生成包络
        overhang = (cargo_w > trailer_w) or (cargo_l > trailer_l)
        cargo = CargoParams(
            length=cargo_l,
            width=cargo_w,
            height=self._f("c_height"),
            cg_height=field("c_cg_height", optional=True) or 0.0,
            is_overhang=overhang,
            cargo_bearing=self.cmb_pad.currentText(),
        )
        trailer = TrailerParams(
            trailer_type=self.cmb_trailer_type.currentText(),
            column_type="单纵列",
            length=self._f("t_length"), width=self._f("t_width"),
            deck_height=self._f("t_deck_height"),
            axle_count=field("t_axle_count", 1),
            axle_spacing=field("t_axle_spacing", 1.5),
            track_width=self._f("t_track_width"),
            kingpin_to_axle=field("t_kingpin_to_axle", optional=True),
            steering_type="全轮强制转向",
            axle_max_steering_angle_deg=field("t_axle_steer_deg", HYDRAULIC_AXLE_DEFAULT_STEER_DEG),
        )
        # 液压车型只在启用路口附加外摆时使用牵引车宽度，其余必填属性为模型占位值。
        tractor = TractorParams(
            wheelbase=field("tr_wheelbase", 1.0),
            max_steering_angle_deg=field("tr_max_steering_angle_deg", 45.0),
            width=field("tr_width", 1.0),
            track_width=field("tr_track_width", optional=True),
            fifth_wheel_pos=field("tr_fifth_wheel_pos", 0.0),
        )
        road_type = self.cmb_road.currentText()
        road_fields = dict(
            road_type=road_type,
            highway_grade=self.cmb_grade.currentText(),
            superelevation=field("superelevation", optional=True),
            vertical_clearance_mode=self.cmb_clearance_mode.currentData(),
            vertical_clearance=field("vertical_clearance"),
        )
        if "clearance_structure_type" not in inactive:
            structure = self.cmb_clearance_structure.currentText()
            if not structure:
                raise ValueError("请选择顶部障碍类型")
            road_fields["clearance_structure_type"] = structure
        if road_type == "圆弧弯道":
            road = RoadParams(arc=ArcCorner(outer_radius=self._f_arc("outer_radius"),
                                            lane_width=self._f_arc("lane_width")), **road_fields)
        else:
            direction = self.cmb_turn_direction.currentText()
            left_widths = {
                key: self._number(self._right_edits[key], optional=True)
                for key in ("W_section", "W_lturn") if key not in inactive
            }
            road = RoadParams(right_angle=RightAngleCorner(
                                  entry_width=self._f_right("entry_width"),
                                  exit_width=self._f_right("exit_width"),
                                  corner_radius=self._f_right("corner_radius"),
                                  intersection_angle_deg=90.0,
                                  turn_direction=direction, **left_widths), **road_fields)
        return InputBundle(cargo=cargo, trailer=trailer, tractor=tractor, road=road,
                           max_speed_kmh=self._f("max_speed_kmh"),
                           combined_cg_height=field("combined_cg_height", optional=True))

    def run_calc(self):
        self.action_clear_result()
        try:
            bundle = self.collect_bundle()
        except ValueError as e:
            QMessageBox.warning(self, "输入错误", f"参数采集/校验失败：\n{e}")
            return
        inactive = self._inactive_fields()
        inactive_keys = (
            inactive
            | {f"arc_{key}" for key in inactive.intersection(self._arc_edits)}
            | {f"right_{key}" for key in inactive.intersection(self._right_edits)}
        )
        model_data = {key: value for key, value in self.pack_model_data().items() if key not in inactive_keys}
        self.set_status_badge("running")
        self.lbl_status.repaint()
        self.btn_calc.setEnabled(False)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            res = run_scene(bundle)
        except Exception as e:
            self.set_status_badge("none")
            QMessageBox.critical(self, "仿真失败", f"弯道通过性仿真出错：\n{e}")
            return
        finally:
            self.btn_calc.setEnabled(True)
            QApplication.restoreOverrideCursor()
        res.user_meta = model_data
        self.cur_result = res
        self.btn_export.setEnabled(True)
        self.set_status_badge("pending" if res.pending else "ok" if res.pass_ok else "ng")
        self._refresh_metrics()
        self._render_result(res)
        self._render_sweep_preview()

    def _change_plot_style(self, action):
        style = action.data()
        if style == self._plot_style:
            return
        self._plot_style = style
        self._settings.setValue("drawing/style", style)
        self._update_plot_controls()
        if self.cur_result is not None:
            self._render_sweep_preview()

    def _update_plot_controls(self):
        labels = {"classic": "放大", "3d": "旋转查看", "playback": "打开回放"}
        self.btn_preview.setText(labels[self._plot_style])
        self.btn_save_img.setVisible(self._plot_style != "playback")

    def _render_sweep_preview(self):
        self._orig_pixmap = None
        self.btn_preview.setEnabled(False)
        self.btn_save_img.setEnabled(False)
        try:
            with BytesIO() as image:
                draw_sweep_view(self.cur_result, image, style=self._plot_style)
                pm = QPixmap()
                if not pm.loadFromData(image.getvalue(), "PNG"):
                    raise ValueError("无法读取生成的扫掠图")
        except Exception as e:
            self.img_label.set_preview(placeholder="扫掠图生成失败，可查看计算过程")
            QMessageBox.warning(self, "出图失败", f"扫掠图生成失败：{e}")
        else:
            self._orig_pixmap = pm
            self.img_label.set_preview(pm)
            playback = self._plot_style == "playback"
            self.btn_preview.setEnabled(not playback or len(self.cur_result.playback_frames) > 1)
            self.btn_save_img.setEnabled(not playback)

    def _render_result(self, res):
        sections, html = [], []
        for line in [*res.verdict_lines, "【综合结论】", f"弯道检验结论：{res.verdict}"]:
            if line.startswith("【") and line.endswith("】"):
                title = line[1:-1]
                anchor = f"step-{len(sections)}"
                sections.append(title)
                html.append(f"<h2 style='margin-top:16px;font-size:14pt;color:#18447d;'>"
                            f"<a name='{anchor}'>{title}</a></h2>")
                html.extend(self._playback_result_links(res, title))
            else:
                html.append(f"<p style='margin:0 0 6px;'>{colorize_verdict_html(line)}</p>")
        with QSignalBlocker(self.process_sections):
            self.process_sections.clear()
            self.process_sections.addItems(sections)
            self.process_sections.setCurrentIndex(max(0, self.process_sections.findText(
                self._initial_process_section(res))))
        self.text_result.setHtml(
            "<div style='font-size:10pt;color:#283d59;line-height:1.45;'>"
            + "".join(html) + "</div>")
        self.process_sections.setEnabled(True)
        self._jump_to_step(self.process_sections.currentIndex())

    @staticmethod
    def _playback_result_links(res, title):
        section = "宽度与横向净空" if res.scene == "arc" else "路口扫掠越界"
        if title != section or len(res.playback_frames) < 2:
            return []
        links = []
        if res.scene == "arc" and not res.metrics["sim_unstable"]:
            meters = min(res.metrics["clearance_inner"], res.metrics["clearance_outer"])
            links.append(f"<p>稳态最小净空 <b>{meters:.3f} m</b>（圆弧稳态校核）</p>")
        clearance = res.playback_clearance
        if clearance is not None:
            links.append(f"<p><a href='replay:minimum'>全程回放最小间距 {clearance.meters:.3f} m"
                         f" · {clearance.body}</a><br>回放参考值。从入弯前到出弯后逐帧量测，负值表示越界。</p>")
        contact = right_angle_invasion(res)
        if contact is not None:
            body, depth, _, _ = contact
            label = "车辆" if body == "vehicle" else "货物"
            links.append(f"<p><a href='replay:invasion'>{label}校核侵入 {depth:.3f} m</a>"
                         "（回放定位到最近采样帧）</p>")
        return links

    def _open_playback_link(self, url):
        if url.scheme() == "replay" and url.path() in ("minimum", "invasion"):
            self._open_playback(url.path())

    @staticmethod
    def _initial_process_section(res):
        metrics = res.metrics
        if res.scene == "arc":
            width_section = "宽度与横向净空"
            if not metrics["steering_ok"] or metrics["sim_unstable"]:
                return "轨迹与扫掠量测"
            if not all(metrics[key] for key in ("turn_channel_ok", "swept_width_ok",
                                                "inner_clearance_ok", "outer_clearance_ok")):
                return width_section
        else:
            width_section = "转向宽度校核"
            if not metrics["direction_width_ok"]:
                return width_section
            if not metrics["split_vehicle_ok"] or not metrics["split_cargo_ok"]:
                return "路口扫掠越界"
        if metrics["vert_clearance_ok"] is False:
            return "垂直净空"
        if not metrics["ok"]:
            return "横向稳定性"
        if res.pending:
            return "垂直净空"
        return width_section

    def _jump_to_step(self, row):
        if row >= 0:
            self.text_result.scrollToAnchor(f"step-{row}")

    # ================= 图片操作 =================
    def save_sweep_img(self):
        if self._plot_style == "playback":
            return
        if self._orig_pixmap is None:
            QMessageBox.information(self, "提示", "请先运行检验生成扫掠图。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存扫掠图", "sweep.png",
                                              "PNG 图片 (*.png)")
        if path:
            if not self._orig_pixmap.save(path if path.lower().endswith(".png") else path + ".png"):
                QMessageBox.warning(self, "保存失败", "无法保存扫掠图。")

    def open_image_viewer(self):
        if self._orig_pixmap is None:
            return
        if self._plot_style == "3d":
            dialog = Sweep3DDialog(self.cur_result, self)
        elif self._plot_style == "playback":
            self._open_playback()
            return
        else:
            dialog = ImageViewerDialog(self._orig_pixmap, self)
        try:
            dialog.exec_()
        finally:
            dialog.deleteLater()

    def _open_playback(self, event=None):
        result = self.cur_result
        if result is None or len(result.playback_frames) < 2:
            return
        dialog = SweepPlaybackDialog(result, self)
        try:
            if event is not None:
                dialog.seek_event(event)
            dialog.exec_()
        finally:
            dialog.deleteLater()

    # ================= 模型持久化 =================
    def pack_model_data(self):
        d = {}
        for k, ed in self.edits.items():
            d[k] = ed.text()
        d["pad"] = self.cmb_pad.currentText()
        d["trailer_type"] = self.cmb_trailer_type.currentText()
        d["road_type"] = self.cmb_road.currentText()
        d["turn_direction"] = self.cmb_turn_direction.currentText()
        d["highway_grade"] = self.cmb_grade.currentText()
        d["vertical_clearance_mode"] = self.cmb_clearance_mode.currentData()
        d["clearance_structure_type"] = self.cmb_clearance_structure.currentText()
        for k, ed in self._arc_edits.items():
            d["arc_" + k] = ed.text()
        for k, ed in self._right_edits.items():
            d["right_" + k] = ed.text()
        d["saved_at"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return d

    def unpack_model_data(self, d):
        self.action_clear_result()
        for prefix, edits in (("", self.edits), ("arc_", self._arc_edits), ("right_", self._right_edits)):
            for key, edit in edits.items():
                value = d.get(prefix + key, self._edit_defaults[key])
                edit.setText("" if value is None else str(value))
        for key, combo, default in (
            ("pad", self.cmb_pad, self.cmb_pad.itemText(0)),
            ("trailer_type", self.cmb_trailer_type, TRAILER_TYPES[0]),
            ("road_type", self.cmb_road, ROAD_TYPES[0]),
            ("highway_grade", self.cmb_grade, "三级公路"),
            ("turn_direction", self.cmb_turn_direction, "右转"),
            ("clearance_structure_type", self.cmb_clearance_structure, ""),
        ):
            index = combo.findText(str(d.get(key, default)))
            combo.setCurrentIndex(index if index >= 0 else combo.findText(default))
        mode = self.cmb_clearance_mode.findData(d.get("vertical_clearance_mode", "unknown"))
        self.cmb_clearance_mode.setCurrentIndex(mode if mode >= 0 else 0)
        self._sync_inputs()

    def action_new(self):
        self.unpack_model_data({})
        self._set_model_path("")

    def action_open(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(self, "打开模型", "",
                                              f"模型文件 (*{MODEL_SUFFIX});;所有文件 (*)")
        if path:
            try:
                d = load_model(path)
            except Exception as e:
                QMessageBox.warning(self, "打开失败", f"{e}")
                return
            self.unpack_model_data(d)
            self._set_model_path(path)
            QMessageBox.information(self, "已打开", f"模型已加载：{os.path.basename(path)}")

    def action_save(self):
        if self.current_model_path:
            try:
                save_model(self.pack_model_data(), ensure_suffix(self.current_model_path))
                QMessageBox.information(self, "保存成功", "模型已保存。")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"{e}")
        else:
            self.action_saveas()

    def action_saveas(self):
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "另存为模型", "model" + MODEL_SUFFIX,
                                              f"模型文件 (*{MODEL_SUFFIX})")
        if path:
            path = ensure_suffix(path)
            try:
                save_model(self.pack_model_data(), path)
                self._set_model_path(path)
                QMessageBox.information(self, "保存成功", "模型已保存。")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", f"{e}")

    def action_clear_result(self):
        self.cur_result = None
        self.set_status_badge("none")
        self.text_result.clear()
        self.process_sections.clear()
        self.process_sections.setEnabled(False)
        self._orig_pixmap = None
        self.img_label.set_preview(placeholder=_EMPTY_PREVIEW)
        self.btn_preview.setEnabled(False)
        self.btn_save_img.setEnabled(False)
        self.btn_export.setEnabled(False)
        self._refresh_metrics()

    def action_show_acceptance(self):
        if self._methods_dialog is None:
            self._methods_dialog = self._build_methods()
        self._methods_dialog.show()
        self._methods_dialog.raise_()
        self._methods_dialog.activateWindow()

    # ================= Word 导出（口令保护：EXPORT_PWD）=================
    def export_word(self):
        res = self.cur_result
        if res is None:
            QMessageBox.information(self, "提示", "请先运行弯道检验，再生成报告。")
            return
        try:
            from app.rendering.report_assembler import assemble_and_save
        except ImportError as e:
            QMessageBox.warning(self, "报告模块不可用",
                                f"报告依赖加载失败：{e}\n"
                                "请确认已安装 python-docx / matplotlib。")
            return
        # 导出口令保护（口令见 config.EXPORT_PWD）
        from PyQt5.QtWidgets import QInputDialog, QLineEdit
        pwd, ok = QInputDialog.getText(self, "导出口令", "请输入导出口令：",
                                       QLineEdit.Password)
        if not ok:
            return
        if pwd != EXPORT_PWD:
            QMessageBox.warning(self, "口令错误", "导出口令不正确，已取消导出。")
            return
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "导出Word报告", "弯道检验报告.docx",
                                              "Word 文档 (*.docx)")
        if path:
            try:
                assemble_and_save(res, path)
                QMessageBox.information(self, "导出成功", f"报告已生成：{path}")
            except Exception as e:
                QMessageBox.warning(self, "导出失败", f"{e}")
