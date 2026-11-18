from types import SimpleNamespace

import pytest
from PyQt5.QtCore import QPoint, QRect, Qt
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QDialog, QFrame, QLineEdit

from app.ui import login


@pytest.mark.parametrize("width,height", [(1600, 900), (640, 328), (480, 280)])
def test_login_form_fits_the_screens_available_area(monkeypatch, width, height):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication([])
    available = QRect(40, 20, width, height)
    screen = SimpleNamespace(availableGeometry=lambda: available)
    monkeypatch.setattr(QGuiApplication, "screenAt", lambda _: screen)
    dialog = login.LoginDialog()
    try:
        dialog.show()
        app.processEvents()
        assert available.contains(dialog.frameGeometry())
        if width == 1600:
            assert (dialog.width(), dialog.height()) == (1060, 740)
        card = dialog.findChild(QFrame, "loginCard")
        for widget in (dialog.edt_user, dialog.edt_pwd, dialog.btn_show_password,
                       dialog.login_error, dialog.btn_login):
            assert widget.isVisible()
            assert card.rect().contains(QRect(widget.mapTo(card, QPoint()), widget.size()))
    finally:
        dialog.close()
        app.processEvents()


def test_login_can_retry_inline_and_hide_or_show_the_password(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(login, "LOGIN_USERS", {"driver": "test-password"})
    app = QApplication.instance() or QApplication([])
    dialog = login.LoginDialog()
    try:
        dialog.show()
        app.processEvents()
        dialog.edt_user.setText("driver")
        QTest.keyClick(dialog.edt_user, Qt.Key_Return)
        assert dialog.edt_pwd.hasFocus() and not dialog.login_error.text()
        assert dialog.edt_pwd.echoMode() == QLineEdit.Password

        QTest.keyClicks(dialog.edt_pwd, "wrong")
        QTest.mouseClick(dialog.btn_show_password, Qt.LeftButton)
        assert dialog.edt_pwd.echoMode() == QLineEdit.Normal
        QTest.mouseClick(dialog.btn_show_password, Qt.LeftButton)
        assert dialog.edt_pwd.echoMode() == QLineEdit.Password
        assert dialog.edt_pwd.text() == "wrong"

        QTest.mouseClick(dialog.btn_show_password, Qt.LeftButton)
        dialog.btn_login.setFocus()
        QTest.keyClick(dialog.btn_login, Qt.Key_Return)
        assert dialog.isVisible() and dialog.login_error.text()
        assert dialog.edt_user.text() == "driver" and dialog.edt_pwd.hasFocus()
        assert not dialog.edt_pwd.text() and dialog.edt_pwd.echoMode() == QLineEdit.Password
        QTest.keyClicks(dialog.edt_pwd, "test-password")
        assert not dialog.login_error.text()
        QTest.keyClick(dialog.edt_pwd, Qt.Key_Return)
        assert dialog.result() == QDialog.Accepted
    finally:
        dialog.close()
        app.processEvents()
