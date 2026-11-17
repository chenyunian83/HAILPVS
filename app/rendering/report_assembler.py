"""Word 弯道通过性检验报告生成。

纯文档构造逻辑，不依赖 Qt 控件；文件选择与保存由界面层负责。
参照 LasingSystem2.0 报告排版惯例：楷体_GB2312、黑色、紧凑表格。
"""
import datetime
import os
import re
import tempfile

from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.oxml.ns import qn
from docx.enum.text import WD_ALIGN_PARAGRAPH

from ..core.passability import SweepScenarioResult
from ..rendering.sweep_view import draw_sweep_view
from ..rendering.verdict_style import split_verdict_segments


# ===== 字体工具（复用参考项目 set_font_kai）=====
def set_font_kai(run, font_size=12, bold=False):
    run.font.name = "楷体_GB2312"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "楷体_GB2312")
    run.font.size = Pt(font_size)
    run.font.color.rgb = RGBColor(0, 0, 0)
    run.font.bold = bold


def _hex_rgb(hexstr: str) -> RGBColor:
    h = hexstr.lstrip("#")
    return RGBColor(int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


def _add_param_table(doc, rows, title_text):
    h = doc.add_heading("", level=1)
    hr = h.add_run(title_text)
    set_font_kai(hr, 14, bold=True)
    tbl = doc.add_table(1, 2)
    hdr = tbl.rows[0].cells
    a = hdr[0].paragraphs[0].add_run("参数名称"); set_font_kai(a, 12, bold=True)
    b = hdr[1].paragraphs[0].add_run("数值"); set_font_kai(b, 12, bold=True)
    for name, val in rows:
        c = tbl.add_row().cells
        rn = c[0].paragraphs[0].add_run(str(name)); set_font_kai(rn, 12)
        rv = c[1].paragraphs[0].add_run(str(val)); set_font_kai(rv, 12)
    return tbl


def _add_para(doc, text, size=12, bold=False):
    p = doc.add_paragraph()
    r = p.add_run(text)
    set_font_kai(r, size, bold=bold)
    return p


_SUBSUP_TAGS = re.compile(r'(<sub>|</sub>|<sup>|</sup>|<b>|</b>|<i>|</i>)')


def _add_formula_para(doc, text, size=12, bold=False):
    """按 <sub>/<sup>/<b>/<i> 分段写入，生成真正带上下标/加粗的 Word 段落。

    与界面富文本共用同一套轻量 HTML 标记；其余实体（&lt; 等）在此解码。
    """
    p = doc.add_paragraph()
    sub = sup = italic = False
    inner_bold = bold
    for tok in _SUBSUP_TAGS.split(text):
        if not tok:
            continue
        if tok == "<sub>":
            sub, sup = True, False
            continue
        if tok == "<sup>":
            sup, sub = True, False
            continue
        if tok == "</sub>" or tok == "</sup>":
            sub = sup = False
            continue
        if tok == "<i>" or tok == "</i>":
            italic = tok == "<i>"
            continue
        if tok == "<b>" or tok == "</b>":
            inner_bold = inner_bold if tok == "</b>" else True
            continue
        tok = tok.replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
        if not tok:
            continue
        # 逐判定词着色（不是整段一色）：同一行可能同时出现"满足"与"不通过"，
        # 整段上色会把"不通过"染成绿色。
        for seg, seg_color in split_verdict_segments(tok):
            r = p.add_run(seg)
            set_font_kai(r, size, bold=inner_bold)
            r.font.italic = italic
            if sub:
                r.font.subscript = True
            if sup:
                r.font.superscript = True
            if seg_color:
                r.font.color.rgb = _hex_rgb(seg_color)
    return p


def assemble_and_save(res: SweepScenarioResult, out_path, *, plot_style="classic"):
    """生成 Word 报告；保留旧的 plot_style 参数，报告始终使用原版详细图。"""
    doc = Document()

    title = doc.add_heading("", level=0)
    t_run = title.add_run("大件运输弯道通过性检验报告")
    set_font_kai(t_run, 18, bold=True)
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER

    tp = doc.add_paragraph()
    tr = tp.add_run(f"报告生成时间：{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    set_font_kai(tr, 12)

    # —— 一、检验方法依据 ——
    from config import ACCEPTANCE_NOTE
    h1 = doc.add_heading("", level=1)
    hr = h1.add_run("一、检验方法依据")
    set_font_kai(hr, 14, bold=True)
    _add_para(doc, ACCEPTANCE_NOTE, 12)

    # —— 二、弯道输入参数 ——
    meta = _assemble_meta(res)
    _add_param_table(doc, meta.get("params", []), "二、弯道输入参数")

    # —— 三、车道与车辆扫掠结果 ——
    # 只保留最关键信息（所需/可供宽度、内外缘净空、结论类）；计算口径与推导过程
    # 不在此表赘述，详见"五、检验结论"的检验过程。
    rows = []
    m = res.metrics
    rows.append(("大件运输尺寸级", m.get('size_grade', '')))
    if res.scene == "arc":
        rows.append(("公路等级", m.get('highway_grade', '')))
        rows.append(("宽度判定方法", m.get("method", "")))
        rows.append(("几何扫掠宽度 (m)", f"{m.get('geometry_swept_w', 0):.3f}"))
        rows.append(("扫空空间宽度 (m)", f"{m.get('W_needed', 0):.3f}"))
        rows.append(("转弯通道宽度 (m)", f"{m.get('W', 0):.3f}"))
        rows.append(("路面宽度 (m)", f"{m.get('road_width', 0):.3f}"))
        rows.append(("横向空间宽度 (m)", f"{m.get('transverse_space_w', 0):.3f}"))
        rows.append(("外缘净空 (m)", f"{m.get('clearance_outer', 0):.3f}"))
        rows.append(("内缘净空 (m)", f"{m.get('clearance_inner', 0):.3f}"))
    else:
        direction = m.get("turn_direction", "右转")
        rows.append(("检验转向", direction))
        rows.append(("最内侧转弯半径 R_m (m)", f"{m.get('R_m', 0):.3f}"))
        rows.append(("扫空空间宽度 W (m)", f"{m.get('W', 0):.3f}"))
        rows.append(("外摆值 ΔB (m)", f"{m.get('deltaB', 0):.3f}"))
        widths = (("物理区净宽", "section_net_w"), ("左转处净宽", "lturn_net_w")) if direction == "左转" else (
            ("入口净宽", "entry_net_w"), ("出口净宽", "exit_net_w"))
        rows.extend((f"{label} (m)", f"{m.get(key, 0):.3f}") for label, key in widths)
        for label, key in (("车辆", "vehicle"), ("货物", "cargo")):
            if not m.get("split_error"):
                rows.append((f"{label}越界面积 / 最大侵入", f"{m.get(f'{key}_outside', 0):.3f} m² / "
                             f"{m.get(f'{key}_depth', 0):.3f} m"))
        if m.get("split_error"):
            rows.append(("扫掠校核", str(m["split_error"])))
    rows.append(("车货总高 (m)", f"{m.get('total_height', 0):.3f}"))
    rows.append(("垂直净空", m["vertical_clearance_status"]))
    rows.append(("横向稳定性", "满足" if m.get('ok') else "不满足"))
    _add_param_table(doc, rows, "三、车道与车辆扫掠结果")

    # —— 四、扫掠轨迹图 ——
    h4 = doc.add_heading("", level=1)
    h4r = h4.add_run("四、弯道扫掠轨迹图")
    set_font_kai(h4r, 14, bold=True)
    img_path = _render_img(res)
    if img_path:
        try:
            doc.add_picture(img_path, width=Inches(6.2))
        finally:
            try:
                os.unlink(img_path)          # 临时 PNG 用完即删，避免 %TEMP% 累积
            except OSError:
                pass
    else:
        # 渲染失败不再静默省略，正文留痕便于排查
        _add_para(doc, "（扫掠轨迹图渲染失败，报告未附图；请检查图形依赖后重新导出。）",
                  12, bold=True)

    # 与界面共用计算过程及分段标题。
    h5 = doc.add_heading("", level=1)
    h5r = h5.add_run("五、计算过程与检验结论")
    set_font_kai(h5r, 14, bold=True)
    for line in res.verdict_lines:
        if line.startswith("【") and line.endswith("】"):
            section = doc.add_heading("", level=2)
            set_font_kai(section.add_run(line[1:-1]), 12, bold=True)
        else:
            _add_formula_para(doc, line, 12)
    _add_para(doc, f"综合评定：该运输车辆（牵引车-挂车-货物）在该弯道工况下可通行性判定为【{res.verdict}】。", 12, bold=True)

    doc.save(out_path)
    return out_path


def _assemble_meta(res: SweepScenarioResult):
    """汇总输入参数到 Word 表。由 MainWindow 在构造时传入轻量元数据。

    此处保持与界面对齐：若未提供 bundle 元数据，则仅回填结果内可得信息。
    """
    # 界面在检验时把输入快照挂到 res.user_meta（dict）
    meta = getattr(res, "user_meta", None) or {}
    params = []
    mapping = [
        ("货物长度(m)", "c_length"),
        ("货物宽度(m)", "c_width"),
        ("货物高度(m)", "c_height"),
        ("重心高度(m)", "c_cg_height"),
        ("挂车长度(m)", "t_length"),
        ("挂车宽度(m)", "t_width"),
        ("车板上表面离地高度(m)", "t_deck_height"),
        ("挂车轴数", "t_axle_count"),
        ("挂车轴距(m)", "t_axle_spacing"),
        ("每轴最大转向角(°)", "t_axle_steer_deg"),
        ("轮距(m)", "t_track_width"),
        ("牵引销至挂车转动中心轴距离(m)", "t_kingpin_to_axle"),
        ("牵引车轴距(m)", "tr_wheelbase"),
        ("牵引车前轮轮距(m)", "tr_track_width"),
        ("最大转向角(°)", "tr_max_steering_angle_deg"),
        ("最高行驶速度(km/h)", "max_speed_kmh"),
        ("弯道超高率", "superelevation"),
        ("车货总体重心离地高(m)", "combined_cg_height"),
        ("顶部障碍类型", "clearance_structure_type"),
        ("实测最小顶部净空(m)", "vertical_clearance"),
    ]
    for name, key in mapping:
        if key in meta and meta[key] not in ("", None):
            params.append((name, meta[key]))
    if not params:
        params.append(("弯道类型", "圆弧弯道" if res.scene == "arc" else "直角弯道"))
    return {"params": params}


def _render_img(res):
    """把扫掠图渲染为临时 PNG 供 Word 内嵌。

    仅捕获"渲染不可用"类异常（缺依赖/后端问题）；程序性错误（如 KeyError）
    不吞，便于暴露真实缺陷。失败返回 None，由调用方在正文留痕。
    """
    try:
        fd = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        path = fd.name
        fd.close()
        draw_sweep_view(res, path, style="classic")
        return path
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"[报告] 扫掠图渲染失败：{type(exc).__name__}: {exc}")
        return None
