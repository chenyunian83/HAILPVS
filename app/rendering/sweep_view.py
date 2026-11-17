"""原版标注、三维展示和扫掠回放的绘图入口。

对标参考工程图：
  - 白底线框，路口越界区域单独着色；
  - 道路边界 = 黑色粗实线（主结构）；
  - 车辆轮廓 = 蓝色中实线（构件）；
  - 扫掠包络与半径构造圆弧 = 蓝色细虚线（构造/测量几何）；
  - 转弯圆心 = 红色十字（关键参考点）；
  - 扫掠半径 R_o / R_i 带引出线标注（仿 R13122 / R4465）。
沿用 LasingSystem2.0 输出约定：Agg 后端、等比例、300dpi。
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.offsetbox import AnchoredText
from matplotlib.patches import Arc, Patch
from mpl_toolkits.axes_grid1.anchored_artists import AnchoredSizeBar

from config import PLOT_DPI
from ..core.passability import SweepScenarioResult
from .plot_geometry import polygon_patches
from .sweep_3d import build_3d_figure

# —— CAD 风格配色：白底 / 黑主结构 / 蓝构件与构造线 / 红关键点 ——
ROAD_COLOR = "#111111"        # 道路边界：黑色粗实线
VEHICLE_COLOR = "#111111"     # 车辆轮廓：黑色中实线
CARGO_COLOR = "#2e9e44"       # 货物轮廓：绿色实线（与车辆/道路区分）
SWEPT_COLOR = "#4d8dd0"       # 扫掠包络（整车占道范围）：蓝色细虚线
SWEPT_ARC_COLOR = "#90959a"   # 扫掠曲线（R_o / R_i 两条半径圆弧）：灰色，区别于标注线
DIM_COLOR = "#0d47a1"         # 尺寸/半径/交角等所有标注线统一色：深蓝色（区别于曲线，避开红色）
CENTER_COLOR = "#e02020"      # 关键参考点：红色十字
TEXT_COLOR = "#0d47a1"        # 标注文字：与标注线同色（深蓝）
SPINE_COLOR = "#b8c0c8"       # 坐标轴细线：浅灰
AXLE_COLOR = "#7b3fe4"        # 液压车轴段/转向轮：紫色（区别于机械车黑色轮廓）
CARGO_SWEPT_COLOR = "#2f9e44"  # 货物扫空曲线（R_sf/R_sa/r_s，货物外伸时）：绿色短划线，区别于车辆扫掠灰线
OVERRUN_COLOR = "#c53030"
PLOT_STYLES = {"classic": "原版详细标注图", "3d": "三维展示图", "playback": "扫掠回放图"}

# 构造弧(扫掠 R_o/R_i)相对道路边界的向内留隙(m)：避免半径弧贴近/压过内缘线
_SWEPT_ARC_INSET = 0.5


def _iter_geometries(geom):
    """把 Polygon/MultiPolygon 展开为 (环点表) 迭代器（含内部环）。"""
    if geom is None or geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield list(geom.exterior.coords)
        for interior in geom.interiors:
            yield list(interior.coords)
    elif geom.geom_type == "MultiPolygon":
        for poly in geom.geoms:
            yield list(poly.exterior.coords)
            for interior in poly.interiors:
                yield list(interior.coords)


def _draw_overrun(ax, geometry):
    return _draw_polygons(ax, geometry, facecolor=OVERRUN_COLOR,
                          edgecolor=OVERRUN_COLOR, alpha=0.28, zorder=1)


def _draw_polygons(ax, geometry, **style):
    patches = list(polygon_patches(geometry, **style))
    for patch in patches:
        ax.add_patch(patch)
    return bool(patches)


def _extents(*geoms):
    xs, ys = [], []
    for g in geoms:
        if g is None or g.is_empty:
            continue
        xs0, ys0, xs1, ys1 = g.bounds
        xs += [xs0, xs1]
        ys += [ys0, ys1]
    if not xs:
        return (-10, 10, -10, 10)
    return (min(xs), max(xs), min(ys), max(ys))


def _view_extents(ax, *geoms):
    """视图范围 = 几何 bounds ∪ 已绘制折线数据点 ∪ 文字/标注锚点。

    尺寸线与半径/宽度标注都画在车辆轮廓之外（`_annotate_rect_dims` 最多外移
    3.2+1.7 / 6.6+1.9 m），只用 road/swept 的 bounds 会把它们排除在 xlim/ylim
    之外，最终图只能依赖 `bbox_inches="tight"` 兜底，留白与比例不稳定。
    """
    minx, maxx, miny, maxy = _extents(*geoms)

    def _acc(x, y):
        nonlocal minx, maxx, miny, maxy
        if math.isfinite(x) and math.isfinite(y):
            minx, maxx = min(minx, x), max(maxx, x)
            miny, maxy = min(miny, y), max(maxy, y)

    for ln in ax.lines:                      # 道路/扫掠/车辆/货物/尺寸线等全部折线
        for x, y in zip(ln.get_xdata(), ln.get_ydata()):
            _acc(float(x), float(y))
    for t in ax.texts:                       # 含 ax.annotate 生成的对象（xy/xyann）
        for p in (getattr(t, "xy", None), getattr(t, "xyann", None)):
            if p is not None:
                _acc(float(p[0]), float(p[1]))
        pos = t.get_position()
        _acc(float(pos[0]), float(pos[1]))
    return minx, maxx, miny, maxy


def _plot_ring(ax, coords, color, lw, ls="-", *, zorder=None):
    """闭合折线绘制（首尾相连），供道路/车辆/包络环共用。"""
    xs = [c[0] for c in coords] + [coords[0][0]]
    ys = [c[1] for c in coords] + [coords[0][1]]
    ax.plot(xs, ys, color=color, linewidth=lw, linestyle=ls, zorder=zorder)


def _crosshair(ax, x, y, arm=1.8):
    """红色十字：关键参考点（转弯圆心）。"""
    ax.plot([x - arm, x + arm], [y, y], color=CENTER_COLOR, linewidth=1.2, zorder=5)
    ax.plot([x, x], [y - arm, y + arm], color=CENTER_COLOR, linewidth=1.2, zorder=5)


def _locked_axle_axis_line(ax, frame, center, tol=0.15, color=AXLE_COLOR):
    """液压车：画**锁死轴（δ=0）**的轴线延长线，指向瞬时转向中心。

    依据规范 B.1.2 条文说明：「轮胎随圆弧切线方向转向，转向角度第一轴线和最后一
    轴线为最大，依次到中间**不转向的一轴线**或中间转向的两轴线（锁死轴）」——
    不转向轴的轴线（垂直于轮胎滚动方向的旋转轴线）延长后即为过瞬心的那条线。

    只画这一根：首/末轴是转向轴，其"轮轴方向"是转向后的方向，画成"轴线延长线"
    易被误读；且当前绘图轴位（车板前/中/后内收 1m）与转角 δ（按轴组跨度算）不同源，
    首/末轴延长线会偏离圆心约 2.7 m，只有锁死轴精确过圆心。

    仅当该轴线确实通过转弯圆心（垂距 ≤ tol）时才绘制——稳态圆弧段成立；
    入/出弯直道瞬心在无穷远，自动不画。机械车（铰接尾随，B.1.1）无此构造。
    返回是否已绘制。
    """
    if not frame or len(frame) <= 5 or not frame[5]:
        return False
    info = frame[5]
    psi = float(info.get("psi", 0.0))
    locked = [a for a in info.get("axles", []) if abs(float(a[3])) < 1e-9]
    if not locked:
        return False
    mx, my = float(locked[0][1]), float(locked[0][2])
    nx, ny = -math.sin(psi), math.cos(psi)          # 车板横向＝不转向轴的轴线方向
    cx, cy = center
    # 圆心在轴线上的投影参数：n̂ 无论朝向圆心与否都成立
    t_star = (cx - mx) * nx + (cy - my) * ny
    if math.hypot(mx + t_star * nx - cx, my + t_star * ny - cy) > tol:
        return False                                 # 瞬心不在此轴线上（直道/口径不符）
    half_track = float(locked[0][4])
    t0, t1 = -(half_track + 2.0), t_star           # 外端让开板车半宽；内端**止于转弯圆心**
    if t0 > t1:
        t0, t1 = t1, t0
    ax.plot([mx + t0 * nx, mx + t1 * nx], [my + t0 * ny, my + t1 * ny],
            color=color, linewidth=0.8, linestyle=(0, (7, 3)), zorder=3)
    return True


def _rotated_rect(cx, cy, ang, half_len, half_wid):
    """以 (cx,cy) 为中心、航向 ang(弧度)、长 2·half_len、宽 2·half_wid 的矩形四角。

    局部偏移 (lx,ly) 按航向旋转： (lx·cos−ly·sin, lx·sin+ly·cos)。
    """
    ca, sa = math.cos(ang), math.sin(ang)
    return [(cx + lx * ca - ly * sa, cy + lx * sa + ly * ca)
            for lx, ly in ((-half_len, -half_wid), (half_len, -half_wid),
                           (half_len, half_wid), (-half_len, half_wid))]


def _plot_steered_axle(ax, cx, cy, psi, delta_deg, half_track, n_tires=8,
                       color=AXLE_COLOR, *, zorder=None):
    """绘制液压车一根轴：轴段 + 两侧按 δ 偏转的多轮胎实体（第六章图式）。

    cx/cy 为轴中点；psi 为车板航向；delta_deg 为该轴转向角（首/末朝瞬心偏转、
    闭锁轴保持 ψ 平行即 δ=0，故无需单独的锁死标志）。half_track 为轮距半宽。
    n_tires 为该轴轮胎数，沿轴横向均布于轮距上，每个轮胎沿偏转方向画小矩形。
    """
    tire_len = 1.1                        # 轮胎实体长度（m，沿滚动方向示意）
    ca, sa = math.cos(psi), math.sin(psi)
    nx, ny = -sa, ca                       # 横向(轴)方向
    # 轴段：垂直于车板纵轴
    ax.plot([cx - nx * half_track, cx + nx * half_track],
            [cy - ny * half_track, cy + ny * half_track],
            color=color, linewidth=1.0, zorder=4 if zorder is None else zorder)
    da = math.radians(delta_deg)
    # 每侧轮胎数；沿轴横向均布以 fit 轮距
    per_side = int(n_tires) // 2
    if per_side < 1:
        per_side = 1
    for sgn in (-1.0, 1.0):
        for k in range(per_side):
            # 该侧第 k 个轮胎的横向位置（自轴中点向该侧，1..per_side 等分拟合半轮距）
            frac = (k + 0.5) / per_side
            wx = cx + sgn * nx * half_track * frac
            wy = cy + sgn * ny * half_track * frac
            tire_wid = 2.0 * half_track / per_side * 0.85   # 轮胎宽占比该排间距
            # 轮胎矩形航向 = 车板航向 + 该轴转向角（此前手写四角有 tsa 符号错误，
            # 使矩形随航向畸变、ψ=90/270° 时退化为零面积线段）
            corners = _rotated_rect(wx, wy, psi + da, tire_len / 2.0,
                                    tire_wid / 2.0)
            _plot_ring(ax, corners, color, 1.0, zorder=zorder)


def _annotate_radius(ax, center, radius, span_deg, label, side, road=None,
                     ls="-", label_ang=None, color=DIM_COLOR):
    """构造圆弧 + 引出线半径标注（仿 R13122 / R4465）。

    span_deg 为圆弧张角（度）：从弯道入口(0°)铺到弯道转角，使构造圆弧
    贴合弯道实际转弯段、不伸进出入弯直道。side 取 'out'/'in'，决定文字
    落在弧线外侧（开阔区）还是内侧（圆心岛区）。label_ang 可另行指定
    引出线/文字所在角度，避免与其余标注重叠。color 允许扫掠曲线
    （R_o / R_i）用灰色，道路曲线半径等标注沿用深蓝。

    传入 road 几何时，圆弧只画出落在道路范围内的连续段：这样内半径圆弧
    落在直角弯内角空腔（道路L形切掉的口袋）里的部分会被裁掉，避免图上
    出现“半径标注圆越过道路边界”的误导。
    """
    cxx, cyy = center
    a = math.radians(span_deg if label_ang is None else label_ang)
    if road is not None:
        from shapely.geometry import Point
        n = 240
        run_x, run_y = [], []
        for i in range(n + 1):
            d = span_deg * i / n
            px = cxx + radius * math.cos(math.radians(d))
            py = cyy + radius * math.sin(math.radians(d))
            if road.covers(Point(px, py)):
                run_x.append(px)
                run_y.append(py)
            else:
                if len(run_x) >= 2:
                    ax.plot(run_x, run_y, color=color,
                            linewidth=0.9, linestyle=ls, zorder=3)
                run_x, run_y = [], []
        if len(run_x) >= 2:
            ax.plot(run_x, run_y, color=color, linewidth=0.9,
                    linestyle=ls, zorder=3)
    else:
        ax.add_patch(Arc((cxx, cyy), 2 * radius, 2 * radius,
                         theta1=0.0, theta2=span_deg,
                         color=color, linewidth=0.9,
                         linestyle=ls, zorder=3))
    ex = cxx + radius * math.cos(a)
    ey = cyy + radius * math.sin(a)
    offset = 2.4 if side == "out" else -2.4
    if side == "in" and radius + offset < 0.5:
        # 内径小于 2.4 m 时标注会落到圆心另一侧（半径为负）→ 翻到外侧
        offset = 2.4
    tx = cxx + (radius + offset) * math.cos(a)
    ty = cyy + (radius + offset) * math.sin(a)
    ax.plot([tx, ex], [ty, ey], color=color, linewidth=0.7, zorder=3)
    ax.text(tx, ty, label, color=color, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.5),
            zorder=4)


def _band_angle_avoiding(frames, center, r_in, r_out, turn_deg, label_pad=3.0):
    """为径向宽度标注线挑一个**不与已画车辆轮廓重叠**的角度（度）。

    在 [8°, turn_deg−7°] 内逐度试算：以该角度作从 r_in−0.5 到 r_out+label_pad 的
    径向线段（含引出到文字的段落），若与任一车辆包络/货物多边形都不相交即视为可用；
    在所有可用角度里取**最靠近弧中点**的那个（既不压车、又落在弯道中段）。
    全部角度都被车占住时返回 None——此时宁可不画标注，也不要把尺寸线压在车上
    （窄路口、车板长相对走行半径很大时会出现这种情况，所需宽度仍见结论框）。
    """
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    polys = []
    for f in frames:
        for g in (f[2] if len(f) > 2 else None, f[4] if len(f) > 4 else None):
            if g is not None and not g.is_empty:
                polys.append(g)
    veh = unary_union(polys) if polys else None
    cx, cy = center
    mid = float(turn_deg) * 0.5
    free = []
    for a in range(8, max(9, int(round(turn_deg)) - 6)):
        ar = math.radians(a)
        seg = LineString([(cx + (r_in - 0.5) * math.cos(ar),
                           cy + (r_in - 0.5) * math.sin(ar)),
                          (cx + (r_out + label_pad) * math.cos(ar),
                           cy + (r_out + label_pad) * math.sin(ar))])
        if veh is None or not seg.intersects(veh):
            free.append(float(a))
    if not free:
        return None
    return min(free, key=lambda a: abs(a - mid))


def _annotate_band(ax, center, r_in, r_out, ang_deg, label):
    """径向尺寸线：在 ang_deg 上从 r_in 到 r_out 画双向箭头，标注扫空/转弯通道宽度 W。"""
    cxx, cyy = center
    a = math.radians(ang_deg)

    def _pt(rr):
        return (cxx + rr * math.cos(a), cyy + rr * math.sin(a))
    p0, p1 = _pt(r_in), _pt(r_out)
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="<->", color=DIM_COLOR, lw=1.0,
                                shrinkA=0, shrinkB=0), zorder=3)
    mx, my = _pt((r_in + r_out) / 2.0)
    lx, ly = _pt(r_out + 2.6)
    ax.plot([mx, lx], [my, ly], color=DIM_COLOR, linewidth=0.7, zorder=3)
    ax.text(lx, ly, label, color=DIM_COLOR, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.5), zorder=4)


def _annotate_angle(ax, center, ang_deg, radius, label, start_deg=0.0):
    """CAD 交角标注：以 center 为顶点画一段张角弧，在弧中点外穿出引出线+文字。"""
    cxx, cyy = center
    ax.add_patch(Arc((cxx, cyy), 2 * radius, 2 * radius,
                     theta1=start_deg, theta2=start_deg + ang_deg,
                     color=DIM_COLOR, linewidth=0.9, zorder=3))
    a = math.radians(start_deg + ang_deg * 0.5)
    ex = cxx + radius * math.cos(a)
    ey = cyy + radius * math.sin(a)
    lx = cxx + (radius + 2.2) * math.cos(a)
    ly = cyy + (radius + 2.2) * math.sin(a)
    ax.plot([lx, ex], [ly, ey], color=DIM_COLOR, linewidth=0.7, zorder=3)
    ax.text(lx, ly, label, color=TEXT_COLOR, fontsize=9,
            ha="center", va="center", family="Microsoft YaHei",
            bbox=dict(facecolor="white", edgecolor="none", pad=1.2), zorder=4)


def _annotate_rect_dims(ax, geom, prefix, offset, color=DIM_COLOR, text_off=1.7,
                        rear_anchor=None):
    """CAD 长/宽尺寸标注：长度沿最长边、宽度沿“尾部”短边。

    长度尺寸线沿矩形最长边外移 offset；宽度尺寸线移到车尾侧的短边（即离
    rear_anchor=牵引车形心 最远的一条短边），避免宽度标注压到车头/牵引车。
    offset 为尺寸线相对轮廓的垂直偏移；prefix 区分“车”“货”；数字不带单位。
    """
    pts = list(geom.exterior.coords)[:4]
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    E = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        nx, ny = -uy, ux
        mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
        if nx * (mx - cx) + ny * (my - cy) < 0:   # 取背离形心的外法向
            nx, ny = -nx, -ny
        E.append([a, b, ux, uy, nx, ny, L, mx, my])

    E.sort(key=lambda e: e[6], reverse=True)        # 长度降序
    len_e = E[0]                                   # 最长边 = 尺寸“长”
    short_edges = [e for e in E[1:] if e[6] < len_e[6] - 1e-6]
    if not short_edges:
        width_e = len_e                            # 正方形：长宽同边
    elif rear_anchor is not None:
        # 尾部短边 = 离车头锚点最远的那条短边
        width_e = max(short_edges, key=lambda e:
                      math.hypot(e[7] - rear_anchor[0], e[8] - rear_anchor[1]))
    else:
        width_e = short_edges[0]

    for idx, (e, tag) in enumerate(((len_e, "长"), (width_e, "宽"))):
        if width_e is len_e and idx == 1:
            continue                               # 正方形长宽同边，仅标一条
        a, b, ux, uy, nx, ny, L, _mx, _my = e
        oa = (a[0] + nx * offset, a[1] + ny * offset)
        ob = (b[0] + nx * offset, b[1] + ny * offset)
        ax.plot([a[0], oa[0]], [a[1], oa[1]], color=color, linewidth=0.6, zorder=3)
        ax.plot([b[0], ob[0]], [b[1], ob[1]], color=color, linewidth=0.6, zorder=3)
        ax.annotate("", xy=ob, xytext=oa,
                    arrowprops=dict(arrowstyle="<->", color=color, lw=0.8,
                                    shrinkA=0, shrinkB=0), zorder=3)
        mx, my = (oa[0] + ob[0]) / 2.0, (oa[1] + ob[1]) / 2.0
        tx, ty = mx + nx * text_off, my + ny * text_off
        ax.text(tx, ty, f"{prefix}{tag} {L:.2f}", color=TEXT_COLOR, fontsize=8,
                ha="center", va="center", family="Microsoft YaHei",
                rotation=(math.degrees(math.atan2(uy, ux)) + 90) % 180 - 90,
                bbox=dict(facecolor="white", edgecolor="none", pad=1.0), zorder=4)


def _hydraulic_front_anchor(trailer_geom, frame0, frame1, heading=None):
    """液压车（无牵引车图形）的“车头前端”锚点，使宽度标注稳定落向车尾。

    前向优先取**该帧自身航向** heading（帧携带 axle_geoms 时即 psi，精确）；
    无航向时退化为"首帧→次帧"的弦方向。锚点放在车板前端短边外侧；
    `_annotate_rect_dims` 取距它最远的短边 = 车尾短边。

    注：圆弧弯道现在同时画入弯前/弯道内/出弯后的车，相邻帧不再共线，
    故必须用自身航向而非弦方向，否则宽度标注会落到错误的短边。
    """
    ux, uy = 0.0, 1.0
    if heading is not None:
        ux, uy = math.cos(heading), math.sin(heading)
    elif frame1 is not None:
        dx, dy = frame1[0] - frame0[0], frame1[1] - frame0[1]
        dl = math.hypot(dx, dy)
        if dl > 1e-6:
            ux, uy = dx / dl, dy / dl
    pts = list(trailer_geom.exterior.coords)[:4]
    half_len = max(math.hypot(pts[i][0] - pts[(i + 1) % 4][0],
                              pts[i][1] - pts[(i + 1) % 4][1])
                  for i in range(4)) / 2.0
    c = trailer_geom.centroid
    return (c.x + ux * (half_len + 0.5), c.y + uy * (half_len + 0.5))


def _build_classic(res: SweepScenarioResult, figsize):
    """原 CAD 标注图，保留半径、车货尺寸和液压转向轴标注。"""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_facecolor("white")
    scene = getattr(res, "scene", "arc")
    metrics = res.metrics
    direction = metrics.get("turn_direction")

    show_overrun = _draw_overrun(ax, res.vehicle_overrun)
    show_overrun = _draw_overrun(ax, res.cargo_overrun) or show_overrun
    worst_body = max(("vehicle", "cargo"), key=lambda body: metrics.get(f"{body}_depth", 0))
    depth_point = metrics.get(f"{worst_body}_depth_point")
    if depth_point is not None:
        ax.plot(*depth_point, marker="o", markerfacecolor="none", markeredgecolor=OVERRUN_COLOR,
                markersize=7, markeredgewidth=1.2, zorder=6)
    # Layer 1: 扫掠包络 —— 蓝色细虚线（构造/测量几何）
    if res.swept is not None:
        for ring in _iter_geometries(res.swept):
            _plot_ring(ax, ring, SWEPT_COLOR, 0.9, ls=(0, (5, 3)))

    # Layer 2: 道路边界 —— 黑色粗实线（主结构）
    if res.road is not None:
        for ring in _iter_geometries(res.road):
            _plot_ring(ax, ring, ROAD_COLOR, 1.9)

    # Layer 3: 抽帧车辆轮廓 —— 挂车/牵引车蓝色中实线；货物绿色细线
    #           （无论超宽与否都单独画出货物轮廓，直观反映车货相对关系）
    for frame in res.vehicle_frames:
        _x, _y, geom, trailer_geom, cargo_geom = frame[:5]
        axle_geoms = frame[5] if len(frame) > 5 else None
        geoms = geom.geoms if geom.geom_type == "MultiPolygon" else [geom]
        # 牵引车（合并几何最后一个组件）
        if len(geoms) >= 2:
            _plot_ring(ax, list(geoms[-1].exterior.coords), VEHICLE_COLOR, 1.1)
        # 挂车车板
        _plot_ring(ax, list(trailer_geom.exterior.coords), VEHICLE_COLOR, 1.1)
        # 液压车轴线层（仅当此帧携带 axle_geoms）
        if axle_geoms and axle_geoms.get("axles"):
            psi = axle_geoms.get("psi", 0.0)
            n_tires = int(axle_geoms.get("n_tires") or 8)
            for (aidx, axc, axy, dd, halft, _locked) in axle_geoms["axles"]:
                _plot_steered_axle(ax, axc, axy, psi, dd, halft,
                                   n_tires=n_tires)
        # 货物轮廓（始终显示）
        _plot_ring(ax, list(cargo_geom.exterior.coords), CARGO_COLOR, 1.2)
        ax.plot([_x], [_y], "o", color=VEHICLE_COLOR, markersize=3, zorder=4)

    # —— 车辆/货物尺寸标注：沿“入弯前第一辆车”的外沿画 CAD 长/宽尺寸线 ——
    if res.vehicle_frames:
        _first = res.vehicle_frames[0]       # 第一辆车 = 入弯前的初始轮廓
        _fg = _first[2].geoms if _first[2].geom_type == "MultiPolygon" else [_first[2]]
        if len(_fg) >= 2:
            # 机械车：以牵引车形心为“车头”锚点，宽度标注落在车尾侧、避开车头
            rear_anchor = (_fg[-1].centroid.x, _fg[-1].centroid.y)
        else:
            # 液压车（无车头图形）：用该帧自身航向 psi 合成车头前端锚点，宽度线落向车尾
            _nf = res.vehicle_frames[1] if len(res.vehicle_frames) > 1 else None
            _psi = (_first[5].get("psi")
                    if len(_first) > 5 and _first[5] else None)
            rear_anchor = _hydraulic_front_anchor(_first[3], _first, _nf,
                                                 heading=_psi)
        _annotate_rect_dims(ax, _first[3], "车", 3.2, color=DIM_COLOR,
                            rear_anchor=rear_anchor)
        _annotate_rect_dims(ax, _first[4], "货", 6.6, color=DIM_COLOR,
                            text_off=1.9, rear_anchor=rear_anchor)

    # Layer 4: 转弯圆心红色十字 + 扫掠半径 / 道路曲线半径 标注（数字无单位，统一为 m）
    # 转弯圆心与仿真转角均由计算层给出，渲染层不再硬编码 (0,0)、也不再跨模块
    # 引用 passability 的私有常量（_TURN_DEG / _RA_TURN_DEG）
    _ctr = metrics.get("turn_center") or (0.0, 0.0)
    center = (float(_ctr[0]), float(_ctr[1]))
    _crosshair(ax, center[0], center[1])
    turn_deg = float(metrics.get("turn_deg") or (140.0 if scene == "arc" else 90.0))
    r_o = float(metrics.get("R_o", 0.0) or 0.0)
    r_i = float(metrics.get("R_i", 0.0) or 0.0)
    # 扫掠曲线 = 外/内两条半径圆弧 R_o / R_i，用灰色（区别于深蓝标注线）。
    # 圆弧弯道：道路即整条弯曲带，构造圆弧完整铺满；直角弯道：道路为 L 形带，
    # 扫掠内弧 R_m 会在中段伸进内角空腔而越出路面，故按道路 footprint 裁剪，
    # 使扫掠曲线只落在弯道（道路带）内。
    clip_road = (res.road.buffer(-_SWEPT_ARC_INSET)
                 if (scene == "right_angle" and res.road is not None
                     and not res.road.is_empty) else None)
    if r_o > 0.5:
        _annotate_radius(ax, center, r_o, turn_deg, f"扫掠外径 R_o = {r_o:.3f}",
                         "out", color=SWEPT_ARC_COLOR, road=clip_road)
    if r_i > 0.5:
        r_i_label = (f"最内侧转弯 R_m = {r_i:.3f}" if scene == "right_angle"
                     else f"扫掠内径 R_i = {r_i:.3f}")
        _annotate_radius(ax, center, r_i, turn_deg, r_i_label, "in",
                         color=SWEPT_ARC_COLOR, road=clip_road)

    # —— 货物前/后外侧扫空曲线（第六章公式；仅在货物外伸时绘制）——
    #    不再标注"货物内扫空 r_s"（用户要求：图面只保留前后外扫空曲线）
    cargo_swept = res.metrics.get("cargo_swept")
    if scene == "arc" and isinstance(cargo_swept, dict) and cargo_swept.get("active"):
        csw_ls = (0, (3, 2))                       # 绿色短划线，区别于车辆扫掠(灰)
        # 前/后外扫空给不同引出线角度，避免对称外伸时两条同半径弧标注叠在一起
        out_arcs = (
            (cargo_swept["R_sf"], 50.0, f"货物前外扫空 R_sf = {cargo_swept['R_sf']:.2f}"),
            (cargo_swept["R_sa"], 30.0, f"货物后外扫空 R_sa = {cargo_swept['R_sa']:.2f}"),
        )
        for radius, l_ang, label_ in out_arcs:
            if radius > 0.5 and radius >= r_o - 1e-9:
                _annotate_radius(ax, center, radius, turn_deg, label_, "out",
                                 color=CARGO_SWEPT_COLOR, ls=csw_ls, label_ang=l_ang)

    # —— 圆弧弯道：补画“弯道部分”道路曲线内/外缘半径（区别于整车扫掠半径）——
    if scene == "arc":
        road_outer = float(metrics.get("road_outer", 0.0) or 0.0)
        road_lane = float(metrics.get("road_lane", 0.0) or 0.0)
        road_inner = road_outer - road_lane
        # 曲线外/内缘构造圆弧完整铺满转弯段，但引出文字错开到中途角度，
        # 避免与末端 R_o / R_i 标注重叠。
        if road_outer > 0.5:
            _annotate_radius(ax, center, road_outer, turn_deg,
                             f"道路曲线外缘 = {road_outer:.2f}", "out",
                             ls=(0, (6, 2)), label_ang=38.0)
        if road_inner > 0.5:
            _annotate_radius(ax, center, road_inner, turn_deg,
                             f"道路曲线内缘 = {road_inner:.2f}", "in",
                             ls=(0, (6, 2)), label_ang=38.0)

    # —— 液压车：锁死轴轴线延长线（指向转弯圆心）——
    #    仅液压车有轴层（帧携带 axle_geoms）；机械车（铰接尾随）不画，绘图口径不变。
    #    弯道内每辆车都画一根，且只画到转弯圆心为止（不过冲、不加文字）。
    if any(len(f) > 5 and f[5] for f in res.vehicle_frames):
        for _f in res.vehicle_frames:
            _locked_axle_axis_line(ax, _f, center)

    # —— 扫掠环带宽度标注 ——
    #    标注角取"最空"的径向方位，避免标注线与已画车辆轮廓重叠；
    #    全被车辆占住（窄路口 + 长车板）时不画该标注，改由结论框给出所需宽度。
    mid_ang = _band_angle_avoiding(res.vehicle_frames, center,
                                   r_i, r_o, turn_deg)
    band_w = max(r_o - r_i, 0.0)
    if mid_ang is not None and r_i > 0.5 and r_o > 0.5 and band_w > 0:
        if scene == "arc":
            # 标注即所画环带的几何宽度 R_o−R_i（不加括注）
            _annotate_band(ax, center, r_i, r_o, mid_ang,
                           f"几何扫掠宽度 R_o−R_i = {band_w:.2f} m")
        else:  # right_angle
            _annotate_band(ax, center, r_i, r_o, mid_ang,
                           f"扫掠宽度 W = {band_w:.2f}")

    # —— 直角弯：交角 + 弯道（内角导圆）半径 标注 ——
    #    路口角点由计算层给出（与车辆走行半径解耦）；缺省时退回 r_path 老口径。
    if scene != "arc":
        r_path = float(metrics.get("r_path", 0.0) or 0.0)
        _cp = metrics.get("corner_point") or (r_path, r_path)
        px, py = float(_cp[0]), float(_cp[1])
        if abs(turn_deg - 90.0) < 1e-6:
            eh = float(metrics.get("entry_width", 0.0) or 0.0) / 2.0
            xh = float(metrics.get("exit_width", 0.0) or 0.0) / 2.0
            # 交角：顶点锚在 L 形弯路的外角肘点（两条车道外缘线的交会点，
            # 即外角轴点），张角弧开口朝向弯道内侧，表示入口/出口两直道
            # 在转角处的夹角（区别于内角导圆那个"圆"）。
            _annotate_angle(ax, (px + eh, py + xh), 90.0, 4.5,
                            f"交角 {turn_deg:.0f}°", start_deg=180.0)
            # 弯道内半径：内角导圆的圆弧半径（圆心在内角顶点左下方）
            cr = float(metrics.get("corner_radius", 0.0) or 0.0)
            if cr > 1e-9:
                cxx, cyy = px - eh - cr, py - xh - cr
                ax.add_patch(Arc((cxx, cyy), 2 * cr, 2 * cr,
                                 theta1=0.0, theta2=90.0,
                                 color=DIM_COLOR, linewidth=0.9,
                                 linestyle=(0, (6, 2)), zorder=3))
                a = math.radians(45.0)
                ex = cxx + cr * math.cos(a)
                ey = cyy + cr * math.sin(a)
                # 标注放在**路面之外**：内角导圆的圆心位于 L 形被切掉的内角空腔里，
                # 故把文字沿 45° 收到弧线内侧（空腔内）再引出，避免像原先取
                # cr+2.4 那样正好落在内角顶点、压住路面转角。半径过小时保底留出余量。
                lab_r = max(cr - 2.6, 0.35 * cr, 0.4)
                lx = cxx + lab_r * math.cos(a)
                ly = cyy + lab_r * math.sin(a)
                ax.plot([lx, ex], [ly, ey], color=DIM_COLOR, linewidth=0.7, zorder=3)
                ax.text(lx, ly, f"内角半径 = {cr:.2f}", color=TEXT_COLOR, fontsize=9,
                        ha="center", va="center", family="Microsoft YaHei",
                        bbox=dict(facecolor="white", edgecolor="none", pad=1.2),
                        zorder=4)
        else:
            # 非正交交角：绕转弯圆心标一段“入口→出口”的转角弧
            r_ang = (min(r_i, r_path) * 0.5 if r_i > 0.5 else r_path * 0.5)
            _annotate_angle(ax, center, turn_deg, max(r_ang, 1.5),
                            f"交角 {turn_deg:.0f}°", start_deg=90.0)

    # Layer 5: 检验结论 + 图例（左下角）
    vc = "#80551b" if res.pending else "#2f855a" if res.pass_ok else "#c53030"
    info_lines = [f"检验结论：{res.verdict}" + (f"（{direction}）" if direction else "")]
    if scene == "arc":
        info_lines.append(f"判定依据：{metrics.get('method', '验算法')}")
        if metrics.get("lookup") == "pass":
            info_lines.append("查表宽度条件：满足")
        else:
            for label, key, available, check in (
                ("通道宽度", "turn_channel_w", "road_width", "turn_channel_ok"),
                ("扫空宽度", "W_swept", "transverse_space_w", "swept_width_ok"),
            ):
                info_lines.append(f"{label} {metrics.get(key, 0):.2f} / {metrics.get(available, 0):.2f} m"
                                  f"（{'满足' if metrics.get(check) else '不满足'}）")
        info_lines.append(f"内缘 / 外缘净空 {metrics.get('clearance_inner', 0):.2f} / "
                          f"{metrics.get('clearance_outer', 0):.2f} m，要求 ≥ {metrics.get('sigma', 0):.2f} m")
        if not metrics.get("steering_ok", True):
            info_lines.append("所需转角超过挂车限值")
        if metrics.get("sim_unstable"):
            info_lines.append("超出铰接尾随模型适用范围")
    elif "entry_width" in metrics:
        widths = (("物理区 / 左转处净宽", "section_net_w", "lturn_net_w") if direction == "左转"
                  else ("入口 / 出口净宽", "entry_net_w", "exit_net_w"))
        title, first, second = widths
        info_lines.append(f"{title} {metrics[first]:.2f} / {metrics[second]:.2f} m，"
                          f"要求 ≥ {metrics['required_w']:.2f} m")
        if metrics.get("split_error"):
            info_lines.append("扫掠越界校核未完成")
        else:
            for label, key in (("车辆", "vehicle"), ("货物", "cargo")):
                info_lines.append(f"{label}越界 {metrics[f'{key}_outside']:.3f} m²，"
                                  f"最大侵入 {metrics[f'{key}_depth']:.3f} m")
    if metrics.get("vert_clearance_ok") is False:
        info_lines.append("垂直净空不足")
    elif metrics.get("vertical_clearance_status") == "待核实":
        info_lines.append("顶部净空待核实")
    if metrics.get("ok") is False:
        info_lines.append("横向稳定性不满足")
    info = "\n".join(info_lines)
    x = 0.985 if direction == "右转" else 0.015
    y = 0.97 if scene == "arc" else 0.03
    ax.text(x, y, info, transform=ax.transAxes, verticalalignment="top" if scene == "arc" else "bottom",
            horizontalalignment="right" if direction == "右转" else "left",
            multialignment="left", fontsize=10.0, family="Microsoft YaHei",
            fontweight="bold", color=vc, linespacing=1.35,
            bbox=dict(facecolor="white", alpha=0.9, edgecolor=vc, linewidth=1.0,
                      boxstyle="round,pad=0.4"))

    handles = [
        Line2D([0], [0], color=ROAD_COLOR, lw=1.9, label="道路边界"),
        Line2D([0], [0], color=SWEPT_COLOR, lw=1.0, ls=(0, (5, 3)),
               label="扫掠包络 - 整车全程占道范围"),
        Line2D([0], [0], color=SWEPT_ARC_COLOR, lw=1.0, label="扫掠曲线（R_o / R_i）"),
    ]
    if show_overrun:
        handles.append(Patch(facecolor=OVERRUN_COLOR, alpha=0.28, label="扫掠越界区域"))
    if depth_point is not None:
        handles.append(Line2D([0], [0], color=OVERRUN_COLOR, ls="none", marker="o",
                              markerfacecolor="none", label="最大侵入位置"))
    # 仅列入本图实际绘制的图元（货物扫空曲线仅在货物外伸时绘制；
    # 道路曲线内/外缘半径、交角构造线仅圆弧弯道绘制）
    _cs = metrics.get("cargo_swept")
    if scene == "arc" and isinstance(_cs, dict) and _cs.get("active"):
        handles.append(Line2D([0], [0], color=CARGO_SWEPT_COLOR, lw=1.0,
                              ls=(0, (3, 2)),
                              label="货物扫空曲线（R_sf / R_sa）"))
    handles += [
        Line2D([0], [0], color=VEHICLE_COLOR, lw=1.1, label="车辆轮廓（挂车 / 牵引车）"),
        Line2D([0], [0], color=CARGO_COLOR, lw=1.2, label="货物轮廓"),
    ]
    if any(len(f) > 5 and f[5] for f in res.vehicle_frames):
        handles.append(Line2D([0], [0], color=AXLE_COLOR, lw=1.1,
                              label="液压车轴线/转向轮（δ 朝瞬心）"))
        handles.append(Line2D([0], [0], color=AXLE_COLOR, lw=0.9, ls=(0, (7, 3)),
                              label="锁死轴轴线延长线（指向转弯圆心）"))
    if scene == "arc":
        handles.append(Line2D([0], [0], color=DIM_COLOR, lw=0.9,
                              label="道路曲线内 / 外缘半径"))
    else:
        handles.append(Line2D([0], [0], color=DIM_COLOR, lw=0.9, ls=(0, (6, 2)),
                              label="交角 / 内角半径 构造线"))
    handles += [
        Line2D([0], [0], color=CENTER_COLOR, ls='none', marker='+', markersize=8,
               label="转弯圆心（车辆绕其转弯）"),
        Line2D([0], [0], color=DIM_COLOR, lw=1.0, ls="-",
               label="尺寸标注（扫掠宽度 W / 车·货长宽）"),
    ]
    from matplotlib.font_manager import FontProperties
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.085), ncol=3,
              framealpha=0.94, edgecolor="#444444", labelspacing=0.45,
              prop=dict(family="Microsoft YaHei", size=8),
              title_fontproperties=FontProperties(family="Microsoft YaHei", size=9))

    minx, maxx, miny, maxy = _view_extents(ax, res.road, res.swept)
    xpad = max((maxx - minx) * 0.12, 2.0)
    ypad = max((maxy - miny) * 0.12, 2.0)
    ax.set_xlim(minx - xpad, maxx + xpad)
    ax.set_ylim(miny - ypad, maxy + ypad)
    if direction == "右转":
        # 仿真以左弯为标准坐标，右弯示意镜像显示，横轴刻度同步映射。
        from matplotlib.ticker import FuncFormatter
        pivot_x = metrics["corner_point"][0]
        ax.invert_xaxis()
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _pos: f"{2 * pivot_x - x:g}"))
    ax.set_aspect("equal", "box")
    for spine in ax.spines.values():
        spine.set_color(SPINE_COLOR)
        spine.set_linewidth(0.8)
    ax.tick_params(colors=SPINE_COLOR, labelsize=8, length=3)
    ax.grid(False)
    ax.set_title("大件运输弯道扫掠通过性检验图", fontsize=13,
                 family="Microsoft YaHei", color="#000000")
    ax.set_xlabel("X (m)", fontsize=10, color="#666666")
    ax.set_ylabel("Y (m)", fontsize=10, color="#666666")
    return fig, ax


_OV_INK = "#19334f"
_OV_MUTED = "#526981"
_OV_ROAD = "#edf0f4"
_OV_BORDER = "#718498"
_OV_SWEPT = "#c1ddf5"
_OV_SWEEP_EDGE = "#578cc3"
_OV_CARGO = "#146b5d"
_OV_TOLERANCE = "#967246"
_OV_FONT = "Microsoft YaHei"


def draw_overview_pose(ax, frame, hydraulic):
    """画出一个仿真姿态，并返回可在回放中替换的图元。"""
    patch_start, line_start = len(ax.patches), len(ax.lines)
    _, _, body, deck, cargo = frame[:5]
    _draw_polygons(ax, deck, facecolor="white", edgecolor=_OV_INK,
                   linewidth=1.4, zorder=5)
    _draw_polygons(ax, cargo, facecolor="#e3f4ec", edgecolor=_OV_CARGO,
                   linewidth=1.0, alpha=0.7, zorder=6)
    if not hydraulic and body.geom_type == "MultiPolygon":
        _draw_polygons(ax, body.geoms[-1], facecolor=_OV_INK, edgecolor=_OV_INK,
                       linewidth=1.2, alpha=0.80, zorder=7)
    if hydraulic and len(frame) > 5 and frame[5]:
        axle = frame[5]
        for _, x, y, turn, half_track, _ in axle["axles"]:
            _plot_steered_axle(ax, x, y, axle["psi"], turn, half_track,
                               n_tires=axle["n_tires"], zorder=7)
    return [*ax.patches[patch_start:], *ax.lines[line_start:]]


def _draw_travel_direction(ax, res):
    path = res.travel_path
    if len(path) < 2:
        return
    xs, ys = zip(*path)
    ax.plot(xs, ys, color=_OV_INK, lw=1.1, ls=(0, (3, 3)), zorder=4)
    for fraction in (0.14, 0.48, 0.84):
        end = min(len(path) - 1, max(1, round(fraction * (len(path) - 1))))
        start = max(0, end - max(1, round(len(path) * 0.055)))
        ax.annotate("", xy=path[end], xytext=path[start], zorder=8,
                    arrowprops=dict(arrowstyle="-|>", color=_OV_INK, lw=1.6,
                                    mutation_scale=18, shrinkA=0, shrinkB=0))


def _overview_callout(ax, point, label, color):
    xlo, xhi = ax.get_xlim()
    ylo, yhi = ax.get_ylim()
    dx = 16 if (point[0] - xlo) / (xhi - xlo) < 0.62 else -16
    dy = 21 if point[1] < ylo + (yhi - ylo) * 0.67 else -22
    ax.annotate(label, xy=point, xytext=(dx, dy), textcoords="offset points",
                ha="left" if dx > 0 else "right", va="bottom" if dy > 0 else "top",
                color=color, fontsize=13, family=_OV_FONT, zorder=11,
                bbox=dict(boxstyle="round,pad=.35", fc="white", ec=color, lw=0.8),
                arrowprops=dict(arrowstyle="-", color=color, lw=1.0))


def _overview_arc_clearance(ax, res):
    m = res.metrics
    if m["sim_unstable"]:
        return
    side = min(("inner", "outer"), key=lambda edge: m[f"clearance_{edge}"])
    inner = side == "inner"
    swept_radius = m["R_i"] if inner else m["R_o"]
    road_radius = m["road_outer"] - (m["road_lane"] if inner else 0)
    center = m["turn_center"]
    turn = m["turn_deg"]
    angle = _band_angle_avoiding(res.vehicle_frames, center, m["R_i"], m["R_o"], turn)
    angle = math.radians(angle if angle is not None else turn / 2)
    dx, dy = math.cos(angle), math.sin(angle)
    sweep = (center[0] + swept_radius * dx, center[1] + swept_radius * dy)
    road = (center[0] + road_radius * dx, center[1] + road_radius * dy)
    color = _OV_INK if m[f"{side}_clearance_ok"] else OVERRUN_COLOR
    ax.plot((sweep[0], road[0]), (sweep[1], road[1]), color=color, lw=2.2,
            marker="o", markersize=3.8, zorder=9)
    edge = "内缘" if inner else "外缘"
    _overview_callout(ax, road, f"{edge}稳态净空 {m[f'clearance_{side}']:.3f} m\n"
                               f"要求 ≥ {m['sigma']:.2f} m", color)


def right_angle_invasion(res):
    """返回回放标记的侵入点，优先定位超限的对象。"""
    m = res.metrics
    if res.scene != "right_angle" or m.get("split_error"):
        return None
    candidates = [(key, m[f"{key}_depth"], m[f"{key}_depth_point"], not m[f"split_{key}_ok"])
                  for key in ("vehicle", "cargo") if m.get(f"{key}_depth_point") is not None]
    return max(candidates, key=lambda item: (item[3], item[1]), default=None)


def _overview_right_angle(ax, res):
    m = res.metrics
    handles = []
    for key, overrun in (("vehicle", res.vehicle_overrun), ("cargo", res.cargo_overrun)):
        ok = m[f"split_{key}_ok"]
        hatch = "..." if ok else "////"
        color = _OV_TOLERANCE if ok else OVERRUN_COLOR
        fill = "#faeddb" if ok else "#fae0e0"
        if _draw_polygons(ax, overrun, facecolor=fill, edgecolor=color,
                          hatch=hatch, linewidth=1.1, zorder=9 if ok else 10):
            handle_label = "容差内侵入" if ok else "超限区域"
            if not any(handle.get_label() == handle_label for handle in handles):
                handles.append(Patch(fc=fill, ec=color, hatch=hatch, label=handle_label))
    contact = right_angle_invasion(res)
    if contact is not None:
        key, depth, point, failed = contact
        label = "车辆" if key == "vehicle" else "货物"
        color = OVERRUN_COLOR if failed else _OV_TOLERANCE
        ax.plot(*point, marker="D" if failed else "o", mfc="white", mec=color,
                ms=6, mew=1.5, zorder=11)
        suffix = "" if failed else "（容差内）"
        _overview_callout(ax, point, f"{label}最大侵入 {depth:.3f} m{suffix}", color)
    return handles


def _overview_footer(fig, res):
    m = res.metrics
    if res.scene == "arc":
        width_label = "通道宽度（参考）" if m["lookup"] == "pass" else "转弯通道宽度"
        width_value = f"{m['turn_channel_w']:.3f} m"
        if m["sim_unstable"]:
            detail_label, detail_value = "轨迹校核", "超出铰接模型适用范围"
        elif not m["steering_ok"]:
            detail_label, detail_value = "挂车转向", "所需转角超限"
        else:
            detail_label = "最小稳态横向净空"
            detail_value = f"{min(m['clearance_inner'], m['clearance_outer']):.3f} m"
    else:
        width_label, width_value = "所需净宽", f"{m['required_w']:.3f} m"
        keys = ("section_net_w", "lturn_net_w") if m["turn_direction"] == "左转" else (
            "entry_net_w", "exit_net_w")
        detail_label, detail_value = "最小可用净宽", f"{min(m[key] for key in keys):.3f} m"
    gap = m["vertical_clearance_gap"]
    for x, label, value in ((0.055, width_label, width_value),
                            (0.34, detail_label, detail_value)):
        fig.text(x, 0.113, label, color=_OV_MUTED, fontsize=13, family=_OV_FONT)
        fig.text(x, 0.073, value, color=_OV_INK, fontsize=18, family=_OV_FONT, weight="bold")
    if gap is None:
        fig.text(0.68, 0.113, "顶部净空", color=_OV_MUTED, fontsize=13, family=_OV_FONT)
        fig.text(0.68, 0.073, m["vertical_clearance_status"], color=_OV_INK,
                 fontsize=18, family=_OV_FONT, weight="bold")
    else:
        total = m["total_height"]
        section = fig.add_axes((0.68, 0.048, 0.06, 0.078), xlim=(0, 1), ylim=(0, 1))
        section.axis("off")
        section.fill_between((0.17, 0.83), 0.09, 0.60, fc=_OV_SWEPT, ec=_OV_SWEEP_EDGE)
        section.plot((0.02, 0.98), (0.07, 0.07), color=_OV_BORDER, lw=1.4)
        color = _OV_INK if m["vert_clearance_ok"] else OVERRUN_COLOR
        roof = 0.90 if gap > 0 else 0.60 if gap == 0 else 0.47
        section.plot((0.02, 0.98), (roof, roof), color=_OV_BORDER, lw=1.4, zorder=4)
        if gap < 0:
            section.fill_between((0.17, 0.83), roof, 0.60, fc="#fae0e0", ec=color,
                                 hatch="///", zorder=3)
        elif gap > 0:
            section.annotate("", (0.50, roof), (0.50, 0.61),
                             arrowprops=dict(arrowstyle="<->", color=color, shrinkA=0, shrinkB=0))
        fig.text(0.755, 0.113, "顶部间隙（剖面示意）", color=_OV_MUTED, fontsize=12, family=_OV_FONT)
        fig.text(0.755, 0.080, f"{gap:.3f} m", color=color, fontsize=18,
                 family=_OV_FONT, weight="bold")
        fig.text(0.755, 0.052, f"净空 {total + gap:.2f} m / 总高 {total:.2f} m",
                 color=_OV_MUTED, fontsize=11, family=_OV_FONT)


def build_playback_figure(res, figsize=(12, 9)):
    """绘制静态道路和全程扫掠底图，供回放叠加当前姿态。"""
    fig = Figure(figsize=figsize, facecolor="white")
    ax = fig.add_axes((0.05, 0.24, 0.90, 0.65))
    ax.set_aspect("equal", "box")
    ax.set_axis_off()
    m = res.metrics
    scenario = "圆弧弯道" if res.scene == "arc" else f"直角弯道 · {m['turn_direction']}"
    scope = "液压轴线板车 · 全轮转向（板车及货物）" if res.hydraulic else "机械挂车 · 牵引车与挂车铰接尾随"
    fig.text(0.055, 0.959, f"{scenario}  /  扫掠回放", color=_OV_INK,
             family=_OV_FONT, fontsize=25, weight="bold", va="top")
    fig.text(0.055, 0.905, scope, color=_OV_MUTED, family=_OV_FONT, fontsize=15, va="top")
    badge_color = "#80551b" if res.pending else "#23664d" if res.pass_ok else OVERRUN_COLOR
    fig.text(0.942, 0.945, res.verdict, color=badge_color, family=_OV_FONT, fontsize=16,
             weight="bold", ha="right", va="top",
             bbox=dict(boxstyle="round,pad=.43", fc="#f3f6fa", ec="none"))

    _draw_polygons(ax, res.road, facecolor=_OV_ROAD, edgecolor="none", zorder=1)
    _draw_polygons(ax, res.swept, facecolor=_OV_SWEPT, edgecolor=_OV_SWEEP_EDGE,
                   linewidth=1.0, zorder=2)
    _draw_polygons(ax, res.road, facecolor="none", edgecolor=_OV_BORDER,
                   linewidth=1.4, zorder=3)
    handles = [
        Patch(fc=_OV_ROAD, ec=_OV_BORDER, label="道路边界"),
        Patch(fc=_OV_SWEPT, ec=_OV_SWEEP_EDGE,
              label="板车及货物扫掠" if res.hydraulic else "车货扫掠范围"),
        Line2D([0], [0], color=_OV_INK, ls="--", marker=">", label="行驶方向"),
    ]
    if _draw_polygons(ax, res.cargo_sweep, facecolor="none", edgecolor=_OV_CARGO,
                      linestyle="--", linewidth=1.3, zorder=4):
        handles.append(Line2D([0], [0], color=_OV_CARGO, ls="--", label="外伸货物扫掠"))
    if res.hydraulic:
        handles.append(Line2D([0], [0], color=AXLE_COLOR, lw=1.5, label="液压转向轮"))
    else:
        handles.append(Patch(fc=_OV_INK, ec=_OV_INK, alpha=0.8, label="牵引车"))
    _draw_travel_direction(ax, res)

    x0, x1, y0, y1 = _extents(res.road, res.swept)
    pad = 0.06 * max(x1 - x0, y1 - y0)
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)
    if m.get("turn_direction") == "右转":
        ax.invert_xaxis()
    if res.scene == "arc":
        _overview_arc_clearance(ax, res)
    elif m.get("split_error"):
        note = AnchoredText("扫掠越界校核未完成", loc="upper left", frameon=False,
                            prop=dict(family=_OV_FONT, color=OVERRUN_COLOR, size=13))
        ax.add_artist(note)
    else:
        handles += _overview_right_angle(ax, res)
    span = max(x1 - x0, y1 - y0)
    scale = 20 if span >= 100 else 10 if span >= 50 else 5
    ax.add_artist(AnchoredSizeBar(ax.transData, scale, f"{scale} m", "lower left",
                                  pad=.4, sep=4, frameon=False, color=_OV_INK,
                                  fontproperties=dict(family=_OV_FONT, size=12)))
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(.049, .157),
               ncol=min(len(handles), 6), frameon=False, prop=dict(family=_OV_FONT, size=12))
    fig.add_artist(Line2D([.055, .945], [.147, .147], transform=fig.transFigure,
                          color="#d5e0ed", lw=1.0))
    _overview_footer(fig, res)
    return fig, ax


def build_figure(res: SweepScenarioResult, figsize=(12, 10), *, style="classic"):
    """构建指定风格的 (fig, ax)，供预览和各导出入口共用。"""
    if style == "classic":
        return _build_classic(res, figsize)
    if style == "3d":
        return build_3d_figure(res, figsize)
    if style == "playback":
        fig, ax = build_playback_figure(res, figsize)
        if res.playback_frames:
            draw_overview_pose(ax, res.playback_frames[0], res.hydraulic)
        return fig, ax
    raise ValueError(f"未知绘图风格：{style}")


def draw_sweep_view(res: SweepScenarioResult, out_path, dpi=PLOT_DPI,
                    figsize=(12, 10), *, style="classic"):
    """保存指定风格的扫掠图到路径或 PNG 二进制流。"""
    fig, _ = build_figure(res, figsize, style=style)
    try:
        if style == "classic":
            fig.tight_layout()
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight" if style == "classic" else None)
    finally:
        plt.close(fig)
    return out_path
