"""渲染层回归（M11）：图形几何正确性与报告着色一致性。

覆盖：
  - `_rotated_rect` 轮胎/矩形四角随航向不畸变（原手写四角 tsa 符号错误，
    ψ=90°/270° 时退化为零面积线段）；
  - 结论文本判定词着色：同一行同时含"满足"与"不通过"时须逐词着色，
    Word 报告不得把"不通过"染成绿色。
"""
import math
import os
import sys
import warnings

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from app.core.datamodel import (InputBundle, CargoParams, TrailerParams,
                                TractorParams, RoadParams, ArcCorner)
from app.core.passability import run_scene
from app.rendering.sweep_view import _rotated_rect
from app.rendering.verdict_style import (verdict_color, split_verdict_segments,
                                         COLOR_GREEN, COLOR_RED)


def _poly_area(pts):
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


# ---- 轮胎/矩形四角几何 ----
def test_rotated_rect_area_constant_across_headings():
    # 矩形面积应与航向无关（恒定 = 长 × 宽）
    half_len, half_wid = 1.1 / 2.0, 0.616 / 2.0
    want = 1.1 * 0.616
    for deg in range(0, 360, 5):
        pts = _rotated_rect(0.0, 0.0, math.radians(deg), half_len, half_wid)
        assert len(pts) == 4
        assert _poly_area(pts) == pytest.approx(want, abs=1e-9), f"deg={deg}"


def test_rotated_rect_is_rectangle_not_bowtie():
    # 四角须构成真矩形：对边等长、邻边垂直、对角线相等
    half_len, half_wid = 0.55, 0.308
    for deg in (0, 30, 45, 90, 135, 230, 270, 315):
        p = _rotated_rect(3.0, -2.0, math.radians(deg), half_len, half_wid)
        sides = [math.dist(p[i], p[(i + 1) % 4]) for i in range(4)]
        assert sides[0] == pytest.approx(sides[2], abs=1e-9)
        assert sides[1] == pytest.approx(sides[3], abs=1e-9)
        assert {round(sides[0], 9), round(sides[1], 9)} == \
            {round(2 * half_len, 9), round(2 * half_wid, 9)}
        # 邻边垂直
        for i in range(4):
            a = (p[(i + 1) % 4][0] - p[i][0], p[(i + 1) % 4][1] - p[i][1])
            b = (p[(i + 2) % 4][0] - p[(i + 1) % 4][0],
                 p[(i + 2) % 4][1] - p[(i + 1) % 4][1])
            assert a[0] * b[0] + a[1] * b[1] == pytest.approx(0.0, abs=1e-9)


def test_rotated_rect_matches_manual_rotation():
    # 局部角点 (lx,ly) 旋转后须与手算一致（航向 90°：长边沿 y）
    p = _rotated_rect(0.0, 0.0, math.radians(90.0), 3.0, 1.5)
    xs = sorted({round(x, 9) for x, _y in p})
    ys = sorted({round(y, 9) for _x, y in p})
    assert xs == [-1.5, 1.5]
    assert ys == [-3.0, 3.0]


# ---- 判定词着色 ----
def test_verdict_color_none_on_polarity_conflict():
    line = "查表法满足（4.3.1-1），但 4.2.1 内/外缘横向预留不足 → 不通过。"
    assert verdict_color(line) is None          # 极性冲突 → 不整段上色
    assert verdict_color("W = 3.49 m ≤ 路面 7.00 m → 满足") == COLOR_GREEN
    assert verdict_color("W = 8.46 m > 路面 7.00 m → 不满足") == COLOR_RED
    assert verdict_color("弯道类型：圆弧弯道") is None


def test_split_verdict_segments_colors_per_word():
    line = "查表法满足（4.3.1-1），但 4.2.1 内/外缘横向预留不足 → 不通过。"
    segs = split_verdict_segments(line)
    assert "".join(s for s, _c in segs) == line          # 无损切分
    colored = {s: c for s, c in segs if c}
    assert colored["满足"] == COLOR_GREEN
    assert colored["不通过"] == COLOR_RED


def _failing_lookup_bundle():
    """示例1 工况：查表法命中但 4.2.1 内/外缘净空为负 → 行内极性冲突。"""
    return InputBundle(
        cargo=CargoParams(weight_t=60, length=10, width=3.0, height=3.0,
                          cg_height=1.2),
        trailer=TrailerParams(trailer_type="重型机械挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=3, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=30.0, lane_width=7.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


@pytest.mark.parametrize("plot_style", ["3d", "playback"])
def test_word_report_always_uses_original_static_drawing(tmp_path, plot_style):
    from io import BytesIO
    from zipfile import ZipFile

    from app.rendering.report_assembler import assemble_and_save
    from app.rendering.sweep_view import draw_sweep_view

    result = run_scene(_failing_lookup_bundle())
    path = tmp_path / "report.docx"
    assemble_and_save(result, path, plot_style=plot_style)
    with BytesIO() as original, ZipFile(path) as report:
        draw_sweep_view(result, original, style="classic")
        assert report.read("word/media/image1.png") == original.getvalue()


def test_word_report_no_run_mixes_verdict_polarity():
    # Word 报告的每个 run 不得同时含"满足"与"不通过"，且含"不通过"的 run 不得为绿色
    docx = pytest.importorskip("docx")
    from app.rendering.report_assembler import assemble_and_save

    res = run_scene(_failing_lookup_bundle())
    assert res.pass_ok is False
    assert any("不通过" in ln and "满足" in ln for ln in res.verdict_lines)

    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "r.docx")
    assemble_and_save(res, path)
    doc = docx.Document(path)
    assert any(p.style.name == "Heading 2" and p.text == "轨迹与扫掠量测" for p in doc.paragraphs)

    green_rgb = docx.shared.RGBColor(0x2f, 0x85, 0x5a)
    for p in doc.paragraphs:
        for r in p.runs:
            t = r.text or ""
            assert not ("满足" in t and "不通过" in t), f"极性混染: {t!r}"
            if "不通过" in t:
                assert r.font.color.rgb != green_rgb, f"'不通过'被染绿: {t!r}"


# ---- 视图范围须覆盖标注（否则最终图靠 bbox_inches 兜底、比例不稳）----
def _hydraulic_arc_bundle():
    from app.core.datamodel import ArcCorner
    return InputBundle(
        cargo=CargoParams(weight_t=60, length=10, width=3.0, height=3.0,
                          cg_height=1.2),
        trailer=TrailerParams(trailer_type="液压轴线挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=4, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="圆弧弯道",
                        arc=ArcCorner(outer_radius=30.0, lane_width=7.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def _right_angle_bundle():
    from app.core.datamodel import RightAngleCorner
    return InputBundle(
        cargo=CargoParams(weight_t=80, length=10, width=5.0, height=3.0,
                          cg_height=1.2, is_overhang=True),
        trailer=TrailerParams(trailer_type="液压轴线挂车", length=12.0, width=3.0,
                              deck_height=1.2, axle_count=4, axle_spacing=1.5,
                              track_width=2.9),
        tractor=TractorParams(wheelbase=4.0, max_steering_angle_deg=45,
                              length=7.0, width=2.5, fifth_wheel_pos=1.0),
        road=RoadParams(road_type="直角弯道",
                        right_angle=RightAngleCorner(entry_width=18.5,
                                                     exit_width=14.0,
                                                     corner_radius=6.0,
                                                     intersection_angle_deg=90.0),
                        vertical_clearance=7.5, highway_grade="三级公路"),
        max_speed_kmh=10.0)


def _anchors(ax):
    out = []
    for t in ax.texts:
        for p in (getattr(t, "xy", None), getattr(t, "xyann", None)):
            if p is not None:
                out.append((float(p[0]), float(p[1])))
        pos = t.get_position()
        out.append((float(pos[0]), float(pos[1])))
    return out


@pytest.mark.parametrize("factory", [_failing_lookup_bundle,
                                     _hydraulic_arc_bundle,
                                     _right_angle_bundle])
def test_all_annotations_inside_view_limits(factory):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import build_figure, _extents

    res = run_scene(factory())
    fig, ax = build_figure(res)
    try:
        x0, x1 = sorted(ax.get_xlim())
        y0, y1 = ax.get_ylim()
        # 确有标注画在"道路 ∪ 扫掠"之外（尺寸线外移），否则本用例无意义
        bx0, bx1, by0, by1 = _extents(res.road, res.swept)
        anchors = _anchors(ax)
        assert any(not (bx0 <= x <= bx1 and by0 <= y <= by1)
                   for x, y in anchors), "本工况无界外标注，用例失去意义"
        # 而它们必须全部落在视图范围内
        for x, y in anchors:
            assert x0 <= x <= x1 and y0 <= y <= y1, f"标注越出视图: ({x:.2f}, {y:.2f})"
    finally:
        matplotlib.pyplot.close(fig)


def test_playback_clearance_reports_the_most_exposed_body_and_frame():
    from shapely.geometry import box

    from app.core.passability import _measure_playback_clearance

    road = box(0, 0, 20, 10)
    deck = box(4, 7, 8, 8)
    cargo_inside = box(5, 7.5, 7, 8.5)
    cargo_outside = box(5, 8.5, 7, 10.3)
    frames = [(6, 7.5, deck, deck, cargo_inside), (6, 8.5, deck, deck, cargo_outside)]
    inside = _measure_playback_clearance(frames[:1], road)
    assert inside.meters == pytest.approx(1.5)
    assert inside.body == "货物"
    outside = _measure_playback_clearance(frames, road)
    assert outside.frame == 1 and outside.body == "货物"
    assert outside.meters == pytest.approx(-0.3)
    assert outside.body_point[1] == pytest.approx(10.3)
    assert outside.road_point[1] == pytest.approx(10.0)


def test_view_metrics_come_from_result_not_hardcoded():
    # 转弯圆心与仿真转角由计算层 metrics 提供，渲染层不再硬编码/引用私有常量
    res = run_scene(_hydraulic_arc_bundle())
    assert res.metrics["turn_center"] == (0.0, 0.0)
    assert res.metrics["turn_deg"] == pytest.approx(140.0)
    ra = run_scene(_right_angle_bundle()).metrics
    # 直角弯：圆心 = 路口角点 −(r, r)，半径由寻优确定（不再硬编码 (0,0)）
    px, py = ra["corner_point"]
    assert ra["turn_center"] == pytest.approx((px - ra["r_path"], py - ra["r_path"]))
    assert ra["r_path"] > 0.0
    assert ra["turn_deg"] == pytest.approx(90.0)


def test_arc_summary_shows_actual_lookup_instead_of_comparing_geometric_width():
    import matplotlib.pyplot as plt
    from app.rendering.sweep_view import build_figure

    bundle = _hydraulic_arc_bundle()
    bundle.road.arc.lane_width = 5.2
    bundle.road.lateral_clearance = 0.0
    bundle.trailer.axle_spacing = 7.5
    result = run_scene(bundle)
    assert result.metrics["lookup"] == "pass"
    assert result.metrics["turn_channel_w"] > result.metrics["road_width"]
    fig, ax = build_figure(result)
    try:
        summary = next(text.get_text() for text in ax.texts if "检验结论" in text.get_text())
        assert "查表宽度条件：满足" in summary
        assert "所需宽度" not in summary
    finally:
        plt.close(fig)


def test_right_angle_plot_marks_the_checked_overrun():
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch
    from app.rendering.sweep_view import build_figure

    bundle = _right_angle_bundle()
    bundle.road.right_angle.entry_width = 2.5
    result = run_scene(bundle)
    assert result.vehicle_overrun.area > 0.1
    fig, ax = build_figure(result)
    try:
        assert any(isinstance(patch, PathPatch) for patch in ax.patches)
        assert any("车辆越界" in text.get_text() for text in ax.texts)
        assert ax.xaxis_inverted()
    finally:
        plt.close(fig)


@pytest.mark.parametrize("factory", [_failing_lookup_bundle,
                                     _hydraulic_arc_bundle,
                                     _right_angle_bundle])
def test_no_missing_glyph_warnings(factory, tmp_path):
    # 图内文字不得使用 Microsoft YaHei 缺字形的字符（如 U+2082 下标₂、
    # U+2081 等 Unicode 下标），否则会渲染成缺字方块。
    # 图内统一改用 ASCII（如 ΔW2）；需要真下标时只在报告富文本里用 <sub>。
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import draw_sweep_view

    res = run_scene(factory())
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        draw_sweep_view(res, str(tmp_path / "sweep.png"))
    bad = [str(w.message) for w in caught if "missing from font" in str(w.message)]
    assert not bad, f"图内出现缺字形字符：{bad}"


# ---- 液压车锁死轴轴线延长线（指向转弯圆心；机械车不画）----
def _dashed_axle_lines(ax):
    from app.rendering.sweep_view import AXLE_COLOR
    return [ln for ln in ax.lines
            if str(ln.get_color()).lower() == AXLE_COLOR
            and ln.get_linestyle() not in ("-", "solid")]


def _point_line_dist(pt, ln):
    xs, ys = list(ln.get_xdata()), list(ln.get_ydata())
    x1, y1, x2, y2 = xs[0], ys[0], xs[-1], ys[-1]
    dx, dy = x2 - x1, y2 - y1
    return abs(dy * pt[0] - dx * pt[1] + x2 * y1 - y2 * x1) / math.hypot(dx, dy)


@pytest.mark.parametrize("factory,expected", [(_hydraulic_arc_bundle, 2),
                                              (_right_angle_bundle, 1)])
def test_locked_axle_extension_passes_turn_center(factory, expected):
    # 规范 B.1.2："依次到中间不转向的一轴线（锁死轴）"——其轴线延长线即指向瞬心。
    # 弯道内每辆车各画一根，全部须通过转弯圆心，且**只画到圆心为止、不加文字**。
    # 圆弧弯道共 4 辆车，仅弯道内那 2 辆出线；直角弯仍按原 3 姿态，仅弧上那辆出线 → 1 根。
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import build_figure

    res = run_scene(factory())
    assert any(len(f) > 5 and f[5] for f in res.vehicle_frames)   # 有轴层＝液压车
    fig, ax = build_figure(res)
    try:
        lines = _dashed_axle_lines(ax)
        assert len(lines) == expected, f"应画 {expected} 根，实际 {len(lines)} 根"
        center = tuple(res.metrics["turn_center"])
        for ln in lines:
            assert _point_line_dist(center, ln) < 0.05, "延长线未通过转弯圆心"
            # 线的轨迹端须止于圆心（不过冲）
            ends = list(zip(ln.get_xdata(), ln.get_ydata()))
            assert min(math.dist(e, center) for e in ends) < 0.05
        # 图上不再出现"锁死轴"文字
        assert not any("锁死轴" in t.get_text() for t in ax.texts)
    finally:
        matplotlib.pyplot.close(fig)


def test_arc_dimension_annotation_on_entry_vehicle():
    # 车/货长宽尺寸标注须画在**入弯前**那辆（vehicle_frames[0]）上
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import build_figure

    res = run_scene(_hydraulic_arc_bundle())
    assert len(res.vehicle_frames) == 4
    entry_xy = tuple(res.vehicle_frames[0][:2])
    curve_xy = [tuple(f[:2]) for f in res.vehicle_frames[1:3]]
    fig, ax = build_figure(res)
    try:
        labels = [t for t in ax.texts
                  if t.get_text().startswith(("车长", "车宽", "货长", "货宽"))]
        assert len(labels) == 4, f"应有车/货长宽 4 条标注，实际 {len(labels)}"
        for t in labels:
            p = t.get_position()
            d_entry = math.dist(p, entry_xy)
            assert all(d_entry < math.dist(p, c) for c in curve_xy), \
                f"{t.get_text()} 未画在入弯前那辆上"
    finally:
        matplotlib.pyplot.close(fig)


def test_mechanical_drawing_unchanged_no_axle_extension():
    # 机械车（铰接尾随，B.1.1）绘图口径不变：无轴层、无锁死轴延长线
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from app.rendering.sweep_view import build_figure

    res = run_scene(_failing_lookup_bundle())
    assert not any(len(f) > 5 and f[5] for f in res.vehicle_frames)
    fig, ax = build_figure(res)
    try:
        assert _dashed_axle_lines(ax) == []
        assert not any("锁死轴" in t.get_text() for t in ax.texts)
    finally:
        matplotlib.pyplot.close(fig)


@pytest.mark.parametrize("factory, hydraulic", [
    (_failing_lookup_bundle, False),
    (_hydraulic_arc_bundle, True),
])
def test_3d_uses_input_heights_and_vehicle_specific_artists(factory, hydraulic, monkeypatch):
    from matplotlib.colors import to_hex
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    from app.rendering import sweep_3d
    from app.rendering.sweep_view import build_figure

    bundle = factory()
    bundle.trailer.deck_height = 1.37
    bundle.cargo.height = 3.72
    rendered_faces = []
    poly_collection = sweep_3d.art3d.Poly3DCollection

    class RecordingCollection(poly_collection):
        def __init__(self, faces, **style):
            rendered_faces.extend(faces)
            super().__init__(faces, **style)

    monkeypatch.setattr(sweep_3d.art3d, "Poly3DCollection", RecordingCollection)
    fig, ax = build_figure(run_scene(bundle), style="3d")
    assert ax.name == "3d"
    heights = {round(z, 2) for face in rendered_faces for _, _, z in face}
    assert 1.37 in heights and max(heights) == 5.09
    labels = {label.get_text() for label in fig.legends[0].get_texts()}
    assert ("转向轴及轮向" in labels) == hydraulic
    assert ("牵引车平面轮廓" in labels) != hydraulic
    assert any(isinstance(artist, Line3DCollection)
               and to_hex(artist.get_colors()[0]) == sweep_3d._AXLE
               for artist in ax.collections) == hydraulic


@pytest.mark.parametrize("factory", [_hydraulic_arc_bundle, _right_angle_bundle])
def test_band_annotation_does_not_overlap_vehicles(factory):
    # 扫掠宽度标注线（含引出到文字的段落）不得与已画车辆轮廓重叠
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from app.rendering.sweep_view import _band_angle_avoiding

    res = run_scene(factory())
    m = res.metrics
    r_i, r_o = float(m["R_i"]), float(m["R_o"])
    center = tuple(m.get("turn_center") or (0.0, 0.0))
    turn_deg = float(m.get("turn_deg") or 140.0)
    ang = _band_angle_avoiding(res.vehicle_frames, center, r_i, r_o, turn_deg)
    if ang is None:
        # 全角度都被车辆占住（窄路口 + 长车板）→ 渲染层不画该标注，由结论框给数
        return
    ar = math.radians(ang)
    seg = LineString([(center[0] + (r_i - 0.5) * math.cos(ar),
                       center[1] + (r_i - 0.5) * math.sin(ar)),
                      (center[0] + (r_o + 3.0) * math.cos(ar),
                       center[1] + (r_o + 3.0) * math.sin(ar))])
    veh = unary_union([g for f in res.vehicle_frames
                       for g in (f[2], f[4]) if g is not None and not g.is_empty])
    assert not seg.intersects(veh), f"标注线在 {ang}° 与车辆重叠"
    assert 0.0 < ang < turn_deg, f"标注角应落在弯道范围内，实际 {ang}"
