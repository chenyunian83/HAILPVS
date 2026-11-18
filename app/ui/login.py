"""用户登录弹窗，按当前屏幕的可用区域布置表单。"""
from PyQt5.QtWidgets import (QDialog, QLabel, QLineEdit, QPushButton, QShortcut, QToolButton,
                             QVBoxLayout, QHBoxLayout, QFrame, QGraphicsDropShadowEffect, QWidget)
from PyQt5.QtGui import (QLinearGradient, QPalette, QBrush, QColor, QFont, QFontMetrics,
                         QCursor, QGuiApplication, QKeySequence)
from PyQt5.QtCore import Qt

from config import LOGIN_USERS, APP_NAME, APP_VERSION
from .qtutil import _app_font, _primary_screen_geometry, APP_QSS

# ================= 尺寸 =================
DIALOG_W, DIALOG_H = 1060, 740
CARD_W, CARD_H = 700, 560
FIELD_W, FIELD_H = 400, 52
CARD_RADIUS = 20
FIELD_RADIUS = 12
VER_RATIO = 0.6
VER_FLOOR = 9
VER_GAP = 8
# ========================================

COLOR_BG_START = "#0F2C59"
COLOR_BG_END = "#2B6CB0"
COLOR_CARD = "#FFFFFF"
COLOR_TITLE = "#1A365D"
COLOR_COMPANY = "#718096"
COLOR_FOOT = "#A0AEC0"
COLOR_BLUE = "#2B6CB0"
COLOR_BLUE_HOVER = "#3182CE"
COLOR_BLUE_PRESSED = "#2C5282"
COLOR_FOCUS = "#3182CE"
COLOR_FIELD_BG = "#F5F5F7"
COLOR_FIELD_HOVER = "#EFF1F4"
COLOR_PLACEHOLDER = "#A0AEC0"

COMPANY_NAME = "昆明中远海运物流有限公司"


class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}{APP_VERSION} - 用户登录")
        screen = QGuiApplication.screenAt(QCursor.pos())
        available = screen.availableGeometry() if screen else _primary_screen_geometry()
        frame_allowance = min(56, max(40, available.height() // 8))
        self.setFixedSize(min(DIALOG_W, available.width() - 32),
                          min(DIALOG_H, available.height() - frame_allowance))
        self.move(available.center() - self.rect().center())
        self.setStyleSheet(APP_QSS)
        self.setFont(_app_font(10))
        self._apply_gradient()
        self._build_ui()
        self.edt_user.setFocus()
        self.edt_user.returnPressed.connect(self.edt_pwd.setFocus)
        self.edt_pwd.returnPressed.connect(self.check_login)
        self.edt_user.textEdited.connect(self.login_error.clear)
        self.edt_pwd.textEdited.connect(self.login_error.clear)

    def _fit_title_pt(self, card_width, prefer=15, floor=11):
        limit = card_width - 48
        for pt in range(prefer, floor - 1, -1):
            if self._title_width(pt) <= limit:
                return pt
        return floor

    @staticmethod
    def _ver_pt(title_pt):
        return max(VER_FLOOR, round(title_pt * VER_RATIO))

    @classmethod
    def _title_width(cls, pt):
        fm_t = QFontMetrics(QFont("Microsoft YaHei", pt, QFont.Bold))
        fm_v = QFontMetrics(QFont("Microsoft YaHei", cls._ver_pt(pt)))
        return (fm_t.horizontalAdvance(APP_NAME) + VER_GAP
                + fm_v.horizontalAdvance(APP_VERSION))

    def _build_title_row(self, card_width):
        title_pt = self._fit_title_pt(card_width)
        ver_pt = self._ver_pt(title_pt)
        title = QLabel(APP_NAME)
        title.setFont(QFont("Microsoft YaHei", title_pt, QFont.Bold))
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title.setStyleSheet(
            "border: none;"
            f" color: {COLOR_TITLE};"
            f" font-family: 'Microsoft YaHei'; font-size: {title_pt}pt;"
            " font-weight: bold;")
        ver = QLabel(APP_VERSION)
        ver.setFont(QFont("Microsoft YaHei", ver_pt))
        ver.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        ver.setStyleSheet(
            "border: none;"
            f" color: {COLOR_COMPANY};"
            f" font-family: 'Microsoft YaHei'; font-size: {ver_pt}pt;"
            " font-weight: normal;")
        lift = max(0, QFontMetrics(title.font()).descent()
                      - QFontMetrics(ver.font()).descent())
        ver.setContentsMargins(0, 0, 0, lift)
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        title_width = title.sizeHint().width()
        centered = (card_width - title_width) // 2
        room = card_width - title_width - ver.sizeHint().width() - VER_GAP - 12
        row.addSpacing(max(0, min(centered, room)))
        row.addWidget(title)
        row.addSpacing(VER_GAP)
        row.addWidget(ver, alignment=Qt.AlignBottom)
        row.addStretch(1)
        return row

    def _apply_gradient(self):
        c0, c1 = QColor(COLOR_BG_START), QColor(COLOR_BG_END)
        gradient = QLinearGradient(0, 0, self.width(), self.height())
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            s = t * t * (3.0 - 2.0 * t)
            gradient.setColorAt(t, QColor(
                round(c0.red() + (c1.red() - c0.red()) * s),
                round(c0.green() + (c1.green() - c0.green()) * s),
                round(c0.blue() + (c1.blue() - c0.blue()) * s)))
        pal = self.palette()
        pal.setBrush(QPalette.Window, QBrush(gradient))
        self.setPalette(pal)
        self.setAutoFillBackground(True)

    def _build_ui(self):
        card_width = min(CARD_W, self.width() - 32)
        card_height = min(CARD_H, self.height() - 24)
        field_width = min(FIELD_W, card_width - 48)
        density = max(0.0, (card_height - 250) / (CARD_H - 250))

        def spacing(compact, normal):
            return round(compact + density * (normal - compact))

        field_height = spacing(36, FIELD_H)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        card = QFrame()
        card.setObjectName("loginCard")
        card.setFixedSize(card_width, card_height)
        card.setStyleSheet(
            "QFrame#loginCard {"
            f" background-color: {COLOR_CARD};"
            " border: none;"
            f" border-radius: {CARD_RADIUS}px;"
            " }")
        shadow = QGraphicsDropShadowEffect(card)
        shadow.setBlurRadius(64)
        shadow.setColor(QColor(0, 0, 0, 45))
        shadow.setOffset(0, 18)
        card.setGraphicsEffect(shadow)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(0, spacing(8, 44), 0, spacing(8, 28))
        card_layout.setSpacing(0)
        card_layout.addStretch(1)

        company = QLabel(COMPANY_NAME)
        company.setAlignment(Qt.AlignCenter)
        company.setStyleSheet(
            "border: none;"
            f" color: {COLOR_FOOT};"
            " font-family: 'Microsoft YaHei'; font-size: 9pt;")
        card_layout.addWidget(company)
        card_layout.addSpacing(spacing(4, 20))
        card_layout.addLayout(self._build_title_row(card_width))
        card_layout.addSpacing(spacing(8, 52))

        self.edt_user = self._create_field("请输入用户名", field_height)
        self.edt_user.setFixedWidth(field_width)
        card_layout.addWidget(self.edt_user, alignment=Qt.AlignCenter)
        card_layout.addSpacing(spacing(6, 18))
        card_layout.addWidget(self._build_password_row(field_width, field_height), alignment=Qt.AlignCenter)
        self.login_error = QLabel()
        self.login_error.setAccessibleName("登录提示")
        self.login_error.setFixedWidth(field_width)
        self.login_error.setStyleSheet("color: #a12626;")
        self.login_error.setFixedHeight(self.login_error.fontMetrics().height() + 4)
        card_layout.addWidget(self.login_error, alignment=Qt.AlignCenter)
        card_layout.addSpacing(spacing(4, 52 - self.login_error.height()))

        self.btn_login = QPushButton("登 录")
        self.btn_login.setAutoDefault(False)
        for key in (Qt.Key_Return, Qt.Key_Enter):
            shortcut = QShortcut(QKeySequence(key), self.btn_login)
            shortcut.setContext(Qt.WidgetShortcut)
            shortcut.activated.connect(self.btn_login.click)
        self.btn_login.setFixedSize(field_width, field_height)
        self.btn_login.setCursor(Qt.PointingHandCursor)
        self.btn_login.setStyleSheet(f"""
            QPushButton {{
                border: none;
                background-color: {COLOR_BLUE};
                color: #ffffff;
                border-radius: {FIELD_RADIUS}px;
                font-family: 'Microsoft YaHei';
                font-size: 14pt;
                font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {COLOR_BLUE_HOVER}; }}
            QPushButton:pressed {{ background-color: {COLOR_BLUE_PRESSED}; }}
        """)
        self.btn_login.clicked.connect(self.check_login)
        card_layout.addWidget(self.btn_login, alignment=Qt.AlignCenter)
        card_layout.addStretch(1)
        root.addStretch(1)
        root.addWidget(card, alignment=Qt.AlignCenter)
        root.addStretch(1)

    def _build_password_row(self, width, height):
        row = QWidget()
        row.setFixedWidth(width)
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.edt_pwd = self._create_field("请输入密码", height, password=True)
        layout.addWidget(self.edt_pwd, 1)
        self.btn_show_password = QToolButton()
        self.btn_show_password.setText("显示")
        self.btn_show_password.setAccessibleName("显示密码")
        self.btn_show_password.setCheckable(True)
        self.btn_show_password.setFocusPolicy(Qt.StrongFocus)
        self.btn_show_password.setCursor(Qt.PointingHandCursor)
        self.btn_show_password.setFixedSize(56, height)
        self.btn_show_password.setStyleSheet(f"""
            QToolButton {{
                background: {COLOR_FIELD_BG};
                border: 1px solid transparent;
                border-radius: {FIELD_RADIUS}px;
                color: {COLOR_BLUE};
            }}
            QToolButton:hover {{ background: {COLOR_FIELD_HOVER}; }}
            QToolButton:focus {{ border-color: {COLOR_FOCUS}; }}
        """)
        self.btn_show_password.toggled.connect(self._set_password_visible)
        layout.addWidget(self.btn_show_password)
        return row

    def _set_password_visible(self, visible):
        self.edt_pwd.setEchoMode(QLineEdit.Normal if visible else QLineEdit.Password)
        self.btn_show_password.setText("隐藏" if visible else "显示")
        self.btn_show_password.setAccessibleName("隐藏密码" if visible else "显示密码")

    def _create_field(self, placeholder, height, password=False):
        edit = QLineEdit()
        edit.setFixedHeight(height)
        edit.setPlaceholderText(placeholder)
        if password:
            edit.setEchoMode(QLineEdit.Password)
        edit.setStyleSheet(f"""
            QLineEdit {{
                background: {COLOR_FIELD_BG};
                border: 1px solid {COLOR_FIELD_BG};
                border-radius: {FIELD_RADIUS}px;
                padding: 0 18px;
                font-family: 'Microsoft YaHei';
                font-size: 16px;
                color: #1A2B4C;
                selection-background-color: {COLOR_FOCUS};
                selection-color: #ffffff;
            }}
            QLineEdit:hover {{
                background: {COLOR_FIELD_HOVER};
                border: 1px solid {COLOR_FIELD_HOVER};
            }}
            QLineEdit:focus {{
                background: {COLOR_CARD};
                border: 1px solid {COLOR_FOCUS};
            }}
        """)
        role = getattr(QPalette, "PlaceholderText", None)
        if role is not None:
            pal = edit.palette()
            pal.setColor(role, QColor(COLOR_PLACEHOLDER))
            edit.setPalette(pal)
        return edit

    def check_login(self):
        u = self.edt_user.text().strip()
        p = self.edt_pwd.text().strip()
        if u in LOGIN_USERS and LOGIN_USERS[u] == p:
            self.btn_show_password.setChecked(False)
            self.accept()
        else:
            self.edt_pwd.clear()
            self.btn_show_password.setChecked(False)
            self.login_error.setText("用户名或密码输入错误，请重试。")
            self.edt_pwd.setFocus()
