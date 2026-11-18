"""多刚体运动学模型：牵引车(单车模型) + 挂车(铰接约束)。

采用经典铰接单车模型：
  牵引车后轴中心为状态点 (x, y, θ)，转向角 δ。
  挂车与牵引车在鞍座(第五轮)处铰接，挂车航向 ψ 由铰接点速度约束更新：
    ψ̇ = v · sin(θ - ψ) / L_trailer_loop
  挂车参考后点 = 鞍座 - L_loop·(cosψ, sinψ)。
"""
from __future__ import annotations
import math

from config import (MIN_TURN_ANGLE_DEG, AXLE_STEER_ABS_LIMIT_DEG,
                    TURN_SAFE_CLEARANCE_B12, POWER_UNIT_LENGTH)
from .datamodel import InputBundle


def _axle_tire_count(trailer) -> int:
    """每轴轮胎数（绘图层示意，第六章图式）。"""
    from config import TRAILER_TIRE_PER_AXLE
    return int(TRAILER_TIRE_PER_AXLE.get(trailer.trailer_type, 4))


def _rect_into(cx, cy, angle, length, width, pts_out):
    """把以 (cx,cy) 为中心、orientation=angle、长为 length、宽为 width 的
    矩形四角(世界坐标)追加到 pts_out。"""
    ca, sa = math.cos(angle), math.sin(angle)
    hx, hy = length / 2.0, width / 2.0
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    for lx, ly in corners:
        wx = cx + lx * ca - ly * sa
        wy = cy + lx * sa + ly * ca
        pts_out.append((wx, wy))


class TractorTrailerKinematics:
    """牵引车(单车模型) + 挂车(尾随点追逐)。

    挂车参考点(车尾/尾随点)与鞍座间为刚性连杆，尾随点速度 = 牵引销速度
    沿连杆方向的投影(无横向滑动)。该模型天然收敛，不会像纯 psi 积分那样
    在定常转弯中振荡。
    """
    def __init__(self, bundle: InputBundle):
        self.b = bundle
        tr = bundle.tractor
        tl = bundle.trailer
        self.wheelbase = tr.wheelbase
        self.max_delta = math.radians(tr.max_steering_angle_deg)
        self.fifth_wheel = tr.fifth_wheel_pos
        self.loop_len = max(tl.kingpin_to_axle or tl.length, 1e-6)
        self.env_w = max(tl.width, bundle.cargo.width)
        self.body_len = max(tl.length, bundle.cargo.length)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.tx = 0.0      # 挂车尾随点
        self.ty = 0.0
        self.v = 0.0
        self.delta = 0.0

    def reset_pose(self, x=0.0, y=0.0, theta=0.0, psi=None):
        self.x, self.y, self.theta = x, y, theta
        if psi is None:
            psi = theta
        # 尾随点在连杆延长线上，初始与牵引车同向
        self.tx, self.ty = self._tail_along(psi)
        self.delta = 0.0

    def _tail_along(self, psi):
        kx, ky = self.kingpin()
        return (kx - self.loop_len * math.cos(psi),
                ky - self.loop_len * math.sin(psi))

    def kingpin(self):
        return (self.x + self.fifth_wheel * math.cos(self.theta),
                self.y + self.fifth_wheel * math.sin(self.theta))

    def trailer_tail(self):
        return (self.tx, self.ty)

    def psi(self):
        """挂车航向：由尾随点→鞍座方向确定。"""
        kx, ky = self.kingpin()
        return math.atan2(ky - self.ty, kx - self.tx)

    def deck_center(self, psi):
        kx, ky = self.kingpin()
        half_length = self.b.trailer.length / 2.0
        return kx - half_length * math.cos(psi), ky - half_length * math.sin(psi)

    def footprint_poly(self, pts_out):
        """把整车(牵引车+挂车/货物)当前外轮廓矩形(4~8个顶点)追加进 pts_out。"""
        ps = self.psi()
        cx, cy = self.deck_center(ps)
        _rect_into(cx, cy, ps, self.body_len, self.env_w, pts_out)
        _rect_into(self.x, self.y, self.theta, self.wheelbase + 1.0,
                   self.b.tractor.width, pts_out)

    def update_step(self, delta, dt):
        self.delta = max(-self.max_delta, min(self.max_delta, delta))
        v = self.v
        if abs(self.delta) < 1e-9:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            ang_vel = v * math.tan(self.delta) / self.wheelbase
            self.theta += ang_vel * dt
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        # 尾随点追逐：速度 = 牵引销速度沿连杆方向投影(无横向滑动)
        kx, ky = self.kingpin()
        dx, dy = kx - self.tx, ky - self.ty
        dist = math.hypot(dx, dy)
        if dist > 1e-12:
            ux, uy = dx / dist, dy / dist
            vx = v * math.cos(self.theta)
            vy = v * math.sin(self.theta)
            proj = vx * ux + vy * uy
            self.tx += proj * ux * dt
            self.ty += proj * uy * dt
            # 刚性长度约束：重投影到链路长度
            dx, dy = kx - self.tx, ky - self.ty
            d = math.hypot(dx, dy)
            if d > 1e-12:
                self.tx = kx - self.loop_len * dx / d
                self.ty = ky - self.loop_len * dy / d
        return True


def run_profile(kin: TractorTrailerKinematics, profile, v=2.0, dt=0.05,
                draw_step=1, max_steps=40000, include_boundaries=False, phase_ends=None):
    """按转向角时序推演，每段可用 (count, delta_rad, dt) 指定自己的步长。

    返回 (frames, kin)。frames 为采样帧，每帧 (x,y,theta,tx,ty)。
    phase_ends 可收集每段最后一个采样帧的索引；保留精确切换帧时启用 include_boundaries。
    """
    def pose():
        return kin.x, kin.y, kin.theta, kin.tx, kin.ty

    frames = [pose()] if include_boundaries else []
    kin.v = v
    step = 0
    for count, delta, *phase_dt in profile:
        step_dt = phase_dt[0] if phase_dt else dt
        for i in range(count):
            if step >= max_steps:
                if include_boundaries:
                    raise ValueError("扫掠仿真超出最大步数，请核查车辆转角和走行半径。")
                return frames, kin
            kin.update_step(delta, step_dt)
            step += 1
            if include_boundaries:
                if step % draw_step == 0 or i == count - 1:
                    frames.append(pose())
            elif (step - 1) % draw_step == 0:
                frames.append(pose())
        if phase_ends is not None:
            phase_ends.append(len(frames) - 1)
    return frames, kin


def build_trajectory_geoms(kin: TractorTrailerKinematics, frames,
                           sample_rate=1, b12=None):
    """把帧序列恢复为完整位姿，逐帧构建外轮廓几何。

    机械车（b12=None）返回 5 元组：
      (x, y, geom, trailer_geom, cargo_geom)：
      x/y            帧参考点（牵引车状态点，抽帧绘制定位）；
      geom           整车扫掠几何（机械=挂车+货物合并矩形 ∪ 牵引车矩形；
                     液压 b12 时=仅车板+货物包络，不含牵引车，用于包络计算）；
      trailer_geom   挂车车板独立矩形（绘制：蓝色车架）；
      cargo_geom     货物独立矩形（绘制：超宽/超长时黑色轮廓凸出车板可见）。

    液压车（传入 b12，D5/§4.5 直角弯复用瞬心）追加第 6 元 axle_geoms：
      (x, y, geom, trailer_geom, cargo_geom, axle_geoms)，供绘制各轴转向层；
      判定 5.2.1 口径不动（仅可视化）；车头不入 geom，扫掠与圆弧弯口径一致。
    """
    import copy

    from shapely.geometry import Polygon, MultiPolygon

    polys = []
    k = copy.copy(kin)
    for i, (x0, y0, th0, tx0, ty0) in enumerate(frames):
        if i % sample_rate != 0:
            continue
        k.x, k.y, k.theta = x0, y0, th0
        k.tx, k.ty = tx0, ty0
        # 牵引车与挂车各成一个矩形，避免帧内自交
        from . import kinematics as _km
        ps = k.psi()
        cx, cy = k.deck_center(ps)
        env_pts, tr_pts, cg_pts, tb_pts = [], [], [], []
        _km._rect_into(cx, cy, ps, k.body_len, k.env_w, env_pts)          # 挂车+货物合并
        _km._rect_into(cx, cy, ps, k.b.trailer.length, k.b.trailer.width,
                       tr_pts)                                            # 挂车车板
        _km._rect_into(cx, cy, ps, k.b.cargo.length, k.b.cargo.width,
                       cg_pts)                                            # 货物
        if b12 is not None:
            # 液压车：板车全轮转向自带瞬心，车头不入包络/判定（与圆弧弯 D5 口径一致），
            # 扫掠仅车板+货物包络，不并入牵引车
            geom = Polygon(env_pts)
        else:
            _km._rect_into(k.x, k.y, k.theta, k.wheelbase + 1.0,
                           k.b.tractor.width, tb_pts)                     # 牵引车
            geom = MultiPolygon([Polygon(env_pts), Polygon(tb_pts)])
        if b12 is not None:
            tl = k.b.trailer
            rho = b12["sweep_center_dist"]
            na = max(int(tl.axle_count), 2)
            ax = axle_steering_geoms(cx, cy, ps, tl, rho, na=na)
            polys.append((x0, y0, geom, Polygon(tr_pts), Polygon(cg_pts),
                          {"psi": ps, "rho": rho, "axles": ax,
                           "n_tires": _axle_tire_count(tl)}))
        else:
            polys.append((x0, y0, geom, Polygon(tr_pts), Polygon(cg_pts)))
    return polys


# =====================================================================
# B.1.2 液压轴线挂车（多轴多轮液压悬挂挂车）闭式解
# =====================================================================
def hydraulic_turn_radii(bundle: InputBundle, turn_angle_theta=None,
                         actual_turn_radius=None):
    """按规范附录 B.1.2 闭式解求液压悬挂挂车的转弯半径、扫空宽度与转弯通道宽度。

    中间一轴（或中间两轴）锁死不转向、首尾轴全轮强制转向，各轮绕同一瞬时
    转动中心做圆周运动。公式按规范附录 B.1.2 原图复核（图 B.1.2 与式
    B.1.2-1~6，V1.5.0 修正符号对应关系）：
      s     = L轮挂·(首末有效轴序号差)               轴组跨度（沿车板纵向）
      d     = 轮距(track_width)                    图 B.1.2 车板**端部横向**的窄尺寸，
                                                   即式(B.1.2-3)的 La，也是式
                                                   (B.1.2-4/-5)径向腿 (D总∓d)/2 里的 d
      D总   = 车货总宽 max(挂车宽, 货宽)             图 B.1.2 车板端部横向的宽尺寸
      base  = s/(2·tanθ)                           瞬心到**内侧轮迹线**的偏距（B.1.2-2）
      R_min = base − (D总−d)/2                     内侧车体外缘半径（B.1.2-4）
      R_max = √( [base+(D总+d)/2]² + [L·Na/2+C]² )  外侧车体外缘半径（B.1.2-5）
      W_d   = R_max − R_min + σ                    最大扫空空间宽度（B.1.2-6）
      R内(轮迹) = base                              （B.1.2-2）
      R外(轮迹) = √( [base+La]² + [L·(Na−1)/2]² )   （B.1.2-3，La=轮距=d）
      W_z   = R外(轮迹) − R内(轮迹) + 0.5            转弯通道宽度（B.1.2-1）
    C = 动力机组长度(trailer.power_unit_length，缺省 0 → 无 C)；
    σ = TURN_SAFE_CLEARANCE_B12(=0.5)。

    **符号自洽性校验**（这是判断 d/D总 对应的硬判据）：记车板中心走行半径为 R，
    则 base = R − d/2，代入 B.1.2-4/-5 得
      R_min = R − D总/2，  R_max 径向腿 = R + D总/2，
    即退化为"车体按总宽绕圆心做刚体圆周"的初等几何，与
    hydraulic_rigid_sweep_radii() 完全一致；(D总∓d)/2 正是车体相对轮迹线的
    内/外悬伸。若把 D总 当成车板长（旧口径 V1.4.x），(D总∓d)/2 会变成纵向
    长度，W_d 将超过车体对角线（物理不可能）且几乎不随弯道半径变化——实测
    15 m×3 m 板车在 R=100 m 时旧口径仍给出 W_d≈15.6 m（刚体扫空仅 3.3 m）。

    θ = 第一轴线内侧轮胎转向角(D2)。三种来源，优先级：
      1) 显式传入 turn_angle_theta；
      2) 传入 actual_turn_radius（车板中心实际走行半径 R）→ 反算
         θ = atan(s/(2·(R − d/2)))。base 为内侧轮迹线偏距，故先扣除半个轮距；
         避免把"结构最大转角下的最紧转弯固有参数"误用于实际弯道。
      3) 均未给 → 挂车轴结构最大转向角 axle_max_steering_angle_deg（固有最不利口径）。
    θ 受挂车轴结构最大转角与工程上限 AXLE_STEER_ABS_LIMIT_DEG 双重钳制(D6)，
    显式指定角度时用 MIN_TURN_ANGLE_DEG 防除零。按实际半径反算时，
    保留所需转角用于能力校核，可行的大半径使用实际小转角。

    D1 locked_axles 双口径：默认严格守规范(中间锁死不改变 s)；仅当锁死轴
    含首/末轴时走规范外推，s=L(last_active−first_active) 并告警。
    返回字典：R_in(=Rmin)/R_out(=Rmax)/W_swept(=Wd,含σ)/W_turn、轮迹
    R_vehicle_in/R_vehicle_out、power_unit_len、base、theta_deg、d(=轴组跨度 s)、
    track(=轮距)、D_total(=车货总宽)、spec_mode、locked、
    sweep_center_dist(=base,D5 瞬心偏距)、warnings。
    """
    tl = bundle.trailer
    na = max(int(tl.axle_count), 2)
    L = tl.axle_spacing                       # L轮挂：相邻轴线间距
    track = float(tl.track_width)             # B.1.2 的 d / La：轮距
    # B.1.2 的 D总：车货总宽（车板端部横向量取，含超宽货物）
    w_total = max(float(tl.width), float(bundle.cargo.width))

    # D1：首/末轴锁死 → 规范外推；纯中间锁死 → 严格守规范(s 不变)
    locked = sorted({int(a) for a in tl.locked_axles if 1 <= int(a) <= na})
    first, last, spec_mode = 1, na, True
    if (1 in locked) or (na in locked):
        active = [i for i in range(1, na + 1) if i not in locked]
        if active:
            first, last = min(active), max(active)
        spec_mode = False
    d = L * (last - first)
    if d <= 0:                                # active 转向轴<2 等退化：回退规范基准
        d = L * (na - 1)
        spec_mode = True

    # D2/D6/D9：θ 取值优先级 = 显式值 > 实际弯道半径反算 > 挂车轴结构最大转角
    theta_max_deg = min(float(tl.axle_max_steering_angle_deg),
                        AXLE_STEER_ABS_LIMIT_DEG)
    theta_required_deg = None
    if turn_angle_theta is not None:
        theta_deg = min(max(float(turn_angle_theta), MIN_TURN_ANGLE_DEG), theta_max_deg)
    elif actual_turn_radius is not None and float(actual_turn_radius) > 0.0:
        # D9：base 为内侧轮迹线偏距，车板中心走行半径 R 时 base = R − 轮距/2
        base_target = float(actual_turn_radius) - track / 2.0
        theta_required_deg = math.degrees(math.atan2(d, 2.0 * base_target))
        theta_deg = min(theta_required_deg, theta_max_deg)
    else:
        theta_deg = theta_max_deg
    theta = math.radians(theta_deg)

    # 动力机组长度 C（B.1.2-5 根号内横项 +C），界面未给时回退 config 缺省(默认 0 → 无 C)
    power_unit_len = float(getattr(tl, "power_unit_length", None)
                           if getattr(tl, "power_unit_length", None) is not None
                           else POWER_UNIT_LENGTH)

    base = d / (2.0 * math.tan(theta))        # 瞬心至内侧轮迹线偏距（B.1.2-2）

    # —— 车货总体最大扫空空间宽度（B.1.2-4/-5/-6，判定“扫空范围内无障碍”）——
    # 径向腿取车体相对轮迹线的内外悬伸 (D总∓d)/2，d 为轮距、D总 为车货总宽。
    r_min0 = base - (w_total - track) / 2.0                    # B.1.2-4
    # 车货总宽过大（或半径过小）时 (D总−d)/2 > base 会使内缘扫空半径取负(退化)。
    # 负半径非物理(扫空圆越过瞬心)，实测判断失真；取下限并告警，防宽度输入竞态。
    if r_min0 <= 0.0:
        r_min = 0.5
        _rmin_floored = True
    else:
        r_min = r_min0
        _rmin_floored = False
    r_max = math.hypot(base + (w_total + track) / 2.0,
                       L * na / 2.0 + power_unit_len)          # B.1.2-5，含横项+C
    w_swept = r_max - r_min + TURN_SAFE_CLEARANCE_B12          # B.1.2-6，Wd 含 σ

    # —— 转弯通道宽度（轮胎通道，B.1.2-1）：轮迹半径差 + 0.5 ——
    r_veh_in = base                                             # B.1.2-2
    # B.1.2-3 的 La：取轮距（与 B.1.2-4/-5 径向腿的 d 同义）
    la = track
    r_veh_out = math.hypot(base + la, L * (na - 1) / 2.0)       # B.1.2-3
    w_turn = r_veh_out - r_veh_in + TURN_SAFE_CLEARANCE_B12    # B.1.2-1

    warnings_ = []
    if not spec_mode:
        warnings_.append(
            f"首/末轴锁死({locked}):d 按 active 首末={d:.3f}m 外推,属规范外推、保守")
    active_count = len([i for i in range(1, na + 1) if i not in locked])
    if active_count < 2:
        warnings_.append("active 转向轴<2,回退规范基准")
    if _rmin_floored:
        warnings_.append(
            f"车货总宽 D<sub>总</sub>({w_total:g}m) 相对内侧轮迹半径 base({base:g}m) 过大,"
            "扫空内缘半径 B.1.2-4 取负,已取下限 0.5m 参与判定,请核实车宽/弯道半径")

    return {
        "R_in": r_min, "R_out": r_max,
        "W_swept": w_swept, "W_turn": w_turn,
        "R_vehicle_in": r_veh_in, "R_vehicle_out": r_veh_out,
        "power_unit_len": power_unit_len,
        "base": base, "theta_deg": round(theta_deg, 4),
        "theta_required_deg": theta_required_deg, "theta_max_deg": theta_max_deg,
        "d": d, "D_total": w_total, "track": track, "La": la,
        "turn_radius": (float(actual_turn_radius)
                        if actual_turn_radius else None),
        "theta_source": ("explicit" if turn_angle_theta is not None
                         else ("turn_radius" if actual_turn_radius
                               else "axle_max")),
        "spec_mode": spec_mode, "locked": locked,
        "sweep_center_dist": base,      # D5 瞬心横向偏距(轨迹层用)
        "warnings": warnings_,
    }


def hydraulic_rigid_sweep_radii(bundle: InputBundle, turn_radius: float):
    """液压轴线挂车（板车）绕转弯圆心的刚体扫空内/外径——**仅板车+货物，不含牵引车**。

    依据规范 B.1.2 条文说明：「牵引车+多轴多轮液压悬挂挂车的转弯半径由所牵引的
    液压悬挂挂车转弯半径决定，牵引车因车身较短，转弯角度大，基本不增加车辆整体
    转弯半径」。故内/外缘预留（4.2.1）应按板车自身外廓量测，与第六章"自走式挂车"
    模型一致（该章亦明确牵引车对转弯半径可不予考虑）。

    几何：板车绕道路圆心 O 做刚体圆周转动，车板中心位于半径 R 的圆周上；刚体转动
    时航向恒垂直于半径（取切向），故车板+货物包络矩形 (env_len × env_w) 上任一点
    P = C + u·t̂ + v·r̂（u∈±env_len/2，v∈±env_w/2）满足 |P|² = (R+v)² + u²，于是
      R_o（最外缘，外前/外后角）= √( (R + env_w/2)² + (env_len/2)² )
      R_i（最内缘，内侧长边中点）  = R − env_w/2
    返回 (R_o, R_i)。
    """
    tl, cargo = bundle.trailer, bundle.cargo
    env_len = max(float(tl.length), float(cargo.length))
    env_w = max(float(tl.width), float(cargo.width))
    r = float(turn_radius)
    r_out = math.hypot(r + env_w / 2.0, env_len / 2.0)
    r_in = max(r - env_w / 2.0, 0.0)
    return r_out, r_in


# =====================================================================
# 阶段二：液压轴线挂车瞬心轨迹 + 多轴转向绘制（D5）
# =====================================================================
def axle_steering_geoms(base_x, base_y, psi, trailer, rho,
                        na=None, locked_axles=None):
    """绕瞬心转向的轴组几何（第六章定点转向，D5/D8 绘图用）。

    局部车板坐标系：纵轴 e=航向、横轴 n=航向左侧、瞬心在弯内侧(−n)方距纵轴 rho。
    base 为**车板中心** (base_x, base_y)；第 i 轴位于其前向后偏移
    offset_i = length/2 − front_overhang − (i−1)·spacing（即自车板前端向后
    front_overhang+(i−1)·spacing 处），与车板矩形几何一致（D7/D8，轴位精确到
    瞬心坐标并随 coupling 定位）。
    转向角 δ_i（度）按瞬心几何：δ_i = atan( (a_i − a_center)/rho )，其中
    a_i 为该轴到轴组中心的纵向差 —— 首/末轴转角最大且方向相反，指向瞬心；
    锁死轴 δ=0；非锁死轴受 ±axle_max_steering_angle_deg 钳制。

    返回 [(轴序号, 轴中点(cx,cy), δ_i°, 轮距半宽, 是否锁死), ...]（仅绘制用，
    全部轴按 na 计算，不再有未实现的 want 过滤参数）。
    """
    if na is None:
        na = max(int(trailer.axle_count), 2)
    lock = {int(a) for a in (locked_axles if locked_axles is not None
                             else trailer.locked_axles) if 1 <= int(a) <= na}
    spacing = float(trailer.axle_spacing)
    half_track = float(trailer.track_width) / 2.0
    max_delta = min(float(trailer.axle_max_steering_angle_deg),
                    AXLE_STEER_ABS_LIMIT_DEG)

    active = [i for i in range(1, na + 1) if i not in lock] or [1]
    first, last = min(active), max(active)
    # 纵向坐标(向后为正, 首 active 轴=0)：仅用于瞬心转向角计算
    pos = {i: (i - first) * spacing for i in range(1, na + 1)}
    a_center = (pos[first] + pos[last]) / 2.0

    ca, sa = math.cos(psi), math.sin(psi)
    half_len = float(trailer.length) / 2.0
    front_oh = float(trailer.front_overhang)
    rear_oh = float(trailer.rear_overhang)

    # 绘制三组代表轴（第六章图式）：一组靠近车头、一组居中、一组靠近车尾。
    # 界面仅提供 车板长/轴数/轴距（无前/后悬），故默认把三组分铺到车板前/中/后段，
    # 各组各内收 1m；仅当显式给出前/后悬时才按悬长锚定。
    _INSET = 1.0
    front_off = half_len - (front_oh if front_oh > 0.0 else _INSET)
    rear_off = -(half_len - (rear_oh if rear_oh > 0.0 else _INSET))
    mid_off = (front_off + rear_off) / 2.0

    mid_i = (na + 1) // 2
    auto_midlock = (na % 2 == 1) and (mid_i not in lock)

    # 三组：首(first→前)/中(mid_i)/末(last→后)；na 过小(首中末重合)时去重
    keys = []
    for k in (first, mid_i, last):
        if k not in keys:
            keys.append(k)

    all_axles = []
    for i in keys:
        if i == first:
            off = front_off       # 前
        elif i == last:
            off = rear_off        # 后
        else:
            off = mid_off         # 中
        cx = base_x + off * ca
        cy = base_y + off * sa
        # 中间组是枢轴中心行 → δ=0（奇轴中间=自动闭锁、偶轴中间=枢轴不转向）；
        # 首/末组按瞬心几何偏转（前正后负）；锁死轴 δ=0 并标记。
        if i in lock:
            theta_i, is_lock = 0.0, True
        elif i == mid_i and i != first and i != last:
            theta_i, is_lock = 0.0, auto_midlock
        else:
            d = math.degrees(math.atan((a_center - pos[i]) / max(rho, 1e-6)))
            d = max(-max_delta, min(max_delta, d))
            theta_i, is_lock = d, False
        all_axles.append((i, cx, cy, theta_i, half_track, is_lock))
    return all_axles


def hydraulic_arc_trajectory(bundle: InputBundle, b12, radius, turn_deg,
                             straight=6.0, dt=0.05, draw_step=8,
                             center=(0.0, 0.0), *, max_step=None,
                             include_envelope=True, exit_straight=None, turn_frames=None):
    """液压轴线挂车专属刚性圆周轨迹（D5：行为区分可视化）。

    整车刚体绕转弯圆心 O 以 radius 为半径做匀速圆周运动（无铰接尾摆），
    产生比机械铰接尾随更紧密的扫掠；每帧同时计算各轴转向几何供绘制。
    center 为转弯圆心（直角弯需按路口角点平移，圆弧弯为原点）。

    返回 (vgeoms, swept)。vgeoms 元素为 6 元组：
      (x, y, geom, trailer_geom, cargo_geom, axle_geoms)
    axle_geoms = {"psi": ψ, "rho": rho, "axles": [(idx,cx,cy,δ°,半轮距,锁死),...]}
    机械车不调用本函数，仍走 build_trajectory_geoms 铰接尾随。
    """
    tl = bundle.trailer
    R = float(radius)
    rho = float(b12["sweep_center_dist"])          # D5 瞬心横向偏距≈base
    na = max(int(tl.axle_count), 2)
    turn_rad = math.radians(turn_deg)
    ox, oy = float(center[0]), float(center[1])

    frames = []
    # —— 出入弯前后各取一段直线，贴合机械车画法 ——
    line_pts = math.ceil(straight / max_step) if max_step is not None else 5
    # 入弯直道：沿 -Y 驶向 (R, 0)，航向 +90°
    for j in range(line_pts):
        y = -straight + straight * (j / line_pts)
        psi = math.radians(90.0)
        frames.append((R, y, psi, rho, (R, y)))
    # 圆弧段：φ: 0→turn_deg，中心 (R cosφ, R sinφ)，航向 90°+φ
    n = max(int(turn_deg / 3.0), 20)
    if max_step is not None:
        outer_r, _ = hydraulic_rigid_sweep_radii(bundle, R)
        n = max(n, math.ceil(turn_rad * outer_r / max_step))
        if n > 40000:
            raise ValueError("扫掠仿真超出最大步数，请核查车辆转角和走行半径。")
    for k in range(n + 1):
        phi = turn_rad * k / n
        cx, cy = R * math.cos(phi), R * math.sin(phi)
        psi = math.radians(90.0) + phi
        frames.append((cx, cy, psi, rho, (cx, cy)))
    if turn_frames is not None:
        turn_frames.extend((line_pts, line_pts + n))
    # 出弯直道：沿末端切向
    end_psi = math.radians(90.0) + turn_rad
    ex, ey = frames[-1][0], frames[-1][1]
    exit_straight = straight if exit_straight is None else exit_straight
    exit_pts = math.ceil(exit_straight / max_step) if max_step is not None else line_pts
    for j in range(1, exit_pts + 1):
        distance = exit_straight * j / exit_pts
        px = ex + distance * math.cos(end_psi)
        py = ey + distance * math.sin(end_psi)
        frames.append((px, py, end_psi, rho, (px, py)))

    vgeoms = []
    for x, y, psi, rho_f, ctr in frames:
        # 平移到实际转弯圆心（直角弯按路口角点平移；圆弧弯 center=(0,0) 不变）
        x, y = x + ox, y + oy
        ctr = (ctr[0] + ox, ctr[1] + oy)
        geom, tr_g, cg_g = _rigid_hydra_geoms(bundle, x, y, psi)
        ax = axle_steering_geoms(ctr[0], ctr[1], psi, tl, rho_f,
                                 na=na)
        vgeoms.append((x, y, geom, tr_g, cg_g,
                       {"psi": psi, "rho": rho_f, "axles": ax,
                        "n_tires": _axle_tire_count(tl)}))

    from .swept_path import polylist_to_multipoly
    swept = polylist_to_multipoly(vgeoms) if include_envelope else None
    return vgeoms, swept


def _rigid_hydra_geoms(bundle, ctr_x, ctr_y, psi):
    """刚体帧几何：挂车/货物包络框各一个矩形(避免帧内自交)。

    液压轴线板车按 B.1.2 自转模型：车板全轮转向自带瞬心，扫掠/判定均为
    绕车板自身瞬心的刚体圆周，**不并入独立牵引车**——扫掠包络仅含车板+货物，
    车头既不入包络也不参与判定（区别于机械铰接路径 build_trajectory_geoms）。
    """
    from shapely.geometry import Polygon

    tl = bundle.trailer
    cargo = bundle.cargo
    env_len = max(tl.length, cargo.length)
    env_w = max(tl.width, cargo.width)
    tr_pts, cg_pts, env_pts = [], [], []
    _rect_into(ctr_x, ctr_y, psi, tl.length, tl.width, tr_pts)
    _rect_into(ctr_x, ctr_y, psi, cargo.length, cargo.width, cg_pts)
    _rect_into(ctr_x, ctr_y, psi, env_len, env_w, env_pts)
    geom = Polygon(env_pts)
    return geom, Polygon(tr_pts), Polygon(cg_pts)
