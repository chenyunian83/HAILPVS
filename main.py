import sys
import os
import io
import traceback

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime.log")


def log(msg):
    try:
        with io.open(LOG, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


log("=" * 60)
log("START python: " + sys.version.replace("\n", " "))
log("cwd: " + os.getcwd())
log("argv: " + repr(sys.argv))

# 先修复 Qt 平台插件路径，再导入 Qt / 界面
import app.ui.qtutil as qtutil
qtutil._fix_qt_plugins()

try:
    from PyQt5.QtWidgets import QApplication, QDialog
    from app.ui.login import LoginDialog
    from app.ui.main_window import MainWindow
    log("app.ui imported OK")

    app = QApplication(sys.argv)
    log("QApplication created OK")

    # 按系统可用性确定界面字体，避免乱码
    fam = qtutil._pick_font_family()
    if fam:
        qtutil.APP_FONT_FAMILY = fam
        app.setStyleSheet(qtutil.APP_QSS.replace("Microsoft YaHei", fam))
    else:
        app.setStyleSheet(qtutil.APP_QSS)
    app.setFont(qtutil._app_font(10))

    login = LoginDialog()
    log("LoginDialog created OK")

    if login.exec_() == QDialog.Accepted:
        log("login accepted")
        try:
            win = MainWindow()
            win.showMaximized()
            log("MainWindow shown, entering event loop")
            code = app.exec_()
            log("event loop exited code=%s" % code)
            sys.exit(code)
        except Exception:
            log("EXCEPTION in MainWindow:\n" + traceback.format_exc())
            raise
    else:
        log("login cancelled by user")
        sys.exit(0)
except Exception:
    tb = traceback.format_exc()
    log("FATAL EXCEPTION:\n" + tb)
    print("=== PROGRAM ERROR ===", file=sys.stderr)
    print(tb, file=sys.stderr)
    input("\nPress Enter to close the window...")
    sys.exit(1)
