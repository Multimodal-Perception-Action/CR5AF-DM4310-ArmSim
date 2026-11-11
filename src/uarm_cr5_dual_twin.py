"""Single-window read-only digital twin for U-Arm and CR5AF + gripper.

Scene contents:
  left          live seven-servo U-Arm (six joints plus trigger)
  right         virtual CR5 following U-Arm, or measured CR5/DM feedback

Only COM7 reads, TCP/30004 feedback, and guarded DM4310 status reads are used.
No robot enable, motion, clear-error, mode change, servo write, or gripper
action command exists in this program.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import threading
import time
from typing import Any

import mujoco
from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts

from cr5_mujoco_twin import ensure_assets, receive_feedback
from feetech_port import resolve_uarm_port
from uarm_dobot_preview import (
    DEGREES_PER_TICK,
    read_positions,
    read_servo_health,
    shortest_tick_step,
)
from uarm_dobot_teleop import FeedbackStream, require_uarm_health, uarm_health_warnings
from uarm_gripper_binary_follow import read_dm_preflight
from uarm_visual_registration import VisualRegistrationPanel, DisplayRegistration
from uarm_sim_follow import SimFollowMapping


ROOT = Path(__file__).resolve().parents[1]
UARM_MODEL = (
    ROOT / "simulation" / "generated" / "uarm_repaired"
    / "uarm_config2_repaired.urdf"
)
CR5_GRIPPER = ROOT / "simulation" / "generated" / "dual_twin" / "cr5_dm4310_gripper.xml"
CALIBRATION = ROOT / "config" / "uarm_cr5_full_calibration.json"
HEALTH_CONFIG = ROOT / "config" / "uarm_dobot_teleop.json"
UARM_GRIPPER_CONFIG = ROOT / "config" / "uarm_gripper_calibration.json"
DM_CONFIG = ROOT / "config" / "dm4310_gripper_calibration.json"
SIM_CONFIG = ROOT / "config" / "uarm_cr5_simulation.json"
# The only CAD-verified absolute pose is the pose stored in config2.STEP.
# Until a physical-pose registration is completed, q=0 must remain that exact
# assembly pose.  Live encoder deltas are shown relative to it; no invented
# "unfolded home" is allowed.
UARM_DISPLAY_HOME_DEG = [0.0] * 6
UARM_DISPLAY_SCALE = 1.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def open_feedback_socket(host: str) -> socket.socket:
    """Open the receive-only 30004 stream; this function never sends bytes."""
    feedback_socket = socket.create_connection((host, 30004), timeout=3.0)
    feedback_socket.settimeout(1.0)
    return feedback_socket


def set_preview_appearance(spec: mujoco.MjSpec) -> None:
    for geom in spec.geoms:
        geom.rgba = [0.10, 0.90, 0.25, 0.28]
        geom.contype = 0
        geom.conaffinity = 0


def set_solid_appearance(
    spec: mujoco.MjSpec, rgba: list[float], *, group: int
) -> None:
    for geom in spec.geoms:
        geom.rgba = rgba
        geom.group = group
        geom.contype = 0
        geom.conaffinity = 0


def make_uarm_spec() -> mujoco.MjSpec:
    # Config2 geometry with support pillars/beam fixed to base and independent
    # trigger. Original user package is preserved by the offline repair builder.
    spec = mujoco.MjSpec.from_file(str(UARM_MODEL))
    set_solid_appearance(spec, [0.10, 0.42, 0.92, 1.0], group=1)
    return spec


def make_cr5_spec(*, preview: bool) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(ensure_assets()))
    # URDF import sets discardvisual; disable it so the attached tool remains visible.
    spec.compiler.discardvisual = False
    gripper = mujoco.MjSpec.from_file(str(CR5_GRIPPER))
    frame = spec.body("Link6").add_frame(
        name="tool_attach",
        pos=[0.0, 0.0, 0.015],
        euler=[0.0, -math.pi / 2.0, 0.0],
    )
    spec.attach(gripper, prefix="tool_", frame=frame)
    if preview:
        set_preview_appearance(spec)
    else:
        set_solid_appearance(spec, [0.92, 0.46, 0.10, 1.0], group=2)
    return spec


def build_scene() -> tuple[mujoco.MjModel, mujoco.MjData]:
    root = mujoco.MjSpec()
    root.modelname = "U-Arm + CR5AF calibration twin"
    root.compiler.discardvisual = False
    root.worldbody.add_light(
        name="key_light", pos=[0.0, -1.0, 2.3], dir=[0.0, 0.4, -1.0]
    )
    root.worldbody.add_light(
        name="fill_light", pos=[-1.0, 1.0, 1.5], dir=[0.5, -0.5, -1.0],
        diffuse=[0.45, 0.45, 0.45], castshadow=False,
    )
    root.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[2.0, 2.0, 0.05],
        rgba=[0.16, 0.17, 0.19, 1.0],
        contype=0,
        conaffinity=0,
    )
    # The Config2 CAD assembly was exported upside down: its base plate is at
    # the largest Z coordinate. Rotate the complete assembly (not individual
    # links) so the plate rests on the floor and the support pillars rise up.
    # 0.167 m is the base mesh's maximum CAD Z, in metres.
    uarm_frame = root.worldbody.add_frame(
        name="uarm", pos=[-0.72, 0.0, 0.167], quat=[0.0, 1.0, 0.0, 0.0]
    )
    cr5_frame = root.worldbody.add_frame(name="cr5", pos=[0.42, 0.0, 0.0])
    root.attach(make_uarm_spec(), prefix="u_", frame=uarm_frame)
    root.attach(make_cr5_spec(preview=False), prefix="a_", frame=cr5_frame)
    model = root.compile()
    return model, mujoco.MjData(model)


def qpos_address(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"joint missing from combined model: {name}")
    return int(model.jnt_qposadr[joint_id])


def set_joint_rad(
    model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float
) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"joint missing from combined model: {name}")
    if bool(model.jnt_limited[joint_id]):
        low, high = map(float, model.jnt_range[joint_id])
        value = clamp(value, low, high)
    data.qpos[int(model.jnt_qposadr[joint_id])] = value


def set_tool_open_fraction(
    model: mujoco.MjModel, data: mujoco.MjData, prefix: str, fraction: float
) -> None:
    opening = 0.020 * clamp(fraction, 0.0, 1.0)
    data.qpos[qpos_address(model, f"{prefix}tool_left_slide")] = opening
    data.qpos[qpos_address(model, f"{prefix}tool_right_slide")] = -opening


class TwinCanvas:
    """Two independently framed views in one read-only Windows window."""

    WIDTH = 1560
    HEIGHT = 560

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        window_title: str = "U-Arm Config2 + CR5AF read-only digital twin",
        input_reference: dict | None = None,
        sim_controls: bool = False,
        simulation_signs: list[float] | None = None,
        simulation_scales: list[float] | None = None,
    ) -> None:
        import ctypes
        if hasattr(ctypes, "windll"):
            try:
                set_thread_dpi = ctypes.windll.user32.SetThreadDpiAwarenessContext
                set_thread_dpi.argtypes = [ctypes.c_void_p]
                set_thread_dpi.restype = ctypes.c_void_p
                set_thread_dpi(ctypes.c_void_p(-4))  # PER_MONITOR_AWARE_V2
            except (AttributeError, OSError):
                try:
                    ctypes.windll.user32.SetProcessDPIAware()
                except (AttributeError, OSError):
                    pass
        import tkinter as tk
        from PIL import Image, ImageTk

        self.tk = tk
        self.model = model
        self.Image = Image
        self.ImageTk = ImageTk
        self.closed = False
        self.dpi_scale = 1.0
        self.panel_width = self.WIDTH // 2
        model.vis.global_.offwidth = self.panel_width
        model.vis.global_.offheight = self.HEIGHT
        self.renderer = mujoco.Renderer(
            model, height=self.HEIGHT, width=self.panel_width
        )
        self.uarm_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.uarm_camera)
        self.uarm_camera.lookat[:] = [-0.80, -0.03, 0.09]
        self.uarm_camera.distance = 0.62
        self.uarm_camera.azimuth = 40
        self.uarm_camera.elevation = -18
        self.uarm_camera.orthographic = 0

        self.cr5_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.cr5_camera)
        self.cr5_camera.lookat[:] = [0.42, 0.0, 0.52]
        self.cr5_camera.distance = 2.05
        self.cr5_camera.azimuth = 35
        self.cr5_camera.elevation = -20
        self.cr5_camera.orthographic = 0

        self.uarm_option = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.uarm_option)
        self.uarm_option.geomgroup[:] = [True, True, False, False, False, False]
        self.cr5_option = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.cr5_option)
        self.cr5_option.geomgroup[:] = [True, False, True, False, False, False]

        self.root = tk.Tk()
        self.root.tk.call("tk", "scaling", 1.0)
        self.root.title(window_title)
        self.root.configure(bg="#101318")
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)
        self.root.geometry(f"{self.WIDTH}x{self.HEIGHT + 126}+40+40")
        self.fullscreen = False
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._leave_fullscreen)

        header = tk.Frame(self.root, bg="#101318", height=84)
        header.pack(fill="x")
        self.status = tk.Label(
            header, bg="#101318", fg="#4f91ff", justify="left",
            anchor="w", font=("Segoe UI", 12, "bold"), padx=14, pady=8,
        )
        self.status.pack(fill="both", expand=True)
        self.sim_mode = tk.StringVar(value="simulation")
        self.sim_actions = []
        self.pause_text = tk.StringVar(value="暂停仿真")
        self.inspect_axis = tk.IntVar(value=4)
        self.highlight_axis = tk.BooleanVar(value=sim_controls)
        self.inspection_text = tk.StringVar(value='选择关节查看转轴与角度')
        self.sim_signs = list(simulation_signs if simulation_signs is not None else [1.0]*6)
        self.sim_scales = list(simulation_scales if simulation_scales is not None else [1.0]*6)
        self.sim_reverse = tk.BooleanVar(value=self.sim_signs[3]<0)
        self.sim_scale = tk.StringVar(value=f'{self.sim_scales[3]:g}')
        self.sim_target_deg = tk.StringVar(value='0')
        self.latest_actual_joints = [0.0] * 6
        self.latest_virtual_joints = [0.0] * 6
        self.sim_saved_text = tk.StringVar(value='右侧虚拟 CR5 方向（独立于实机）')
        if sim_controls:
            from tkinter import ttk
            controls = ttk.Frame(self.root)
            controls.pack(fill="x", padx=8, pady=3)
            ttk.Radiobutton(controls,text="右侧：实时跟随仿真",variable=self.sim_mode,
                            value="simulation").pack(side="left",padx=4)
            ttk.Radiobutton(controls,text="右侧：实机反馈映射",variable=self.sim_mode,
                            value="actual").pack(side="left",padx=4)
            ttk.Button(controls,text="当前主臂配对实机姿态",
                       command=lambda:self.sim_actions.append('actual')).pack(side="left",padx=4)
            ttk.Button(controls,text="当前主臂配对工作候选",
                       command=lambda:self.sim_actions.append('candidate')).pack(side="left",padx=4)
            ttk.Button(controls,textvariable=self.pause_text,
                       command=lambda:self.sim_actions.append('pause')).pack(side="left",padx=4)
            ttk.Label(controls,text="仅仿真 · 不下发动作").pack(side="left",padx=6)
            inspect = ttk.Frame(self.root)
            inspect.pack(fill='x',padx=8,pady=2)
            ttk.Label(inspect,text='右侧仿真方向：').pack(side='left')
            for index in range(1,7):
                ttk.Radiobutton(inspect,text=f'J{index}',variable=self.inspect_axis,
                                value=index,command=self._sync_sim_reverse).pack(side='left',padx=3)
            ttk.Checkbutton(inspect,text='本轴反向',variable=self.sim_reverse,
                            command=self._change_sim_reverse).pack(side='left',padx=5)
            ttk.Label(inspect,text='倍率').pack(side='left',padx=(8,2))
            ttk.Combobox(inspect,textvariable=self.sim_scale,
                         values=('0.25','0.5','0.75','1','1.25','1.5','2'),
                         width=5,state='readonly').pack(side='left',padx=2)
            ttk.Button(inspect,text='应用倍率',command=self._apply_sim_scale).pack(side='left',padx=3)
            ttk.Label(inspect,text='虚拟 CR5 目标角°').pack(side='left',padx=(8,2))
            ttk.Entry(inspect,textvariable=self.sim_target_deg,width=8).pack(side='left',padx=2)
            ttk.Button(inspect,text='填入 CR5 实测',command=self._fill_actual_target).pack(side='left',padx=3)
            ttk.Button(inspect,text='设为虚拟目标',command=self._apply_virtual_target).pack(side='left',padx=3)
            ttk.Button(inspect,text='保存仿真映射',
                       command=lambda:self.sim_actions.append('save_signs')).pack(side='left',padx=5)
            ttk.Checkbutton(inspect,text='黄色转轴',variable=self.highlight_axis).pack(side='left',padx=5)
            ttk.Label(inspect,textvariable=self.sim_saved_text).pack(side='left',padx=8)
            ttk.Label(self.root,textvariable=self.inspection_text).pack(fill='x',padx=8)
            ttk.Label(self.root,text='倍率和手动目标只修改绿色虚拟 CR5；不写入实机控制配置，不发送机器人命令。',
                      foreground='#a34100').pack(anchor='w',padx=10,pady=(0,3))
        self.registration = VisualRegistrationPanel(self.root, input_reference=input_reference)
        self.registration.pack(fill="x", padx=8, pady=4)
        for child in self.registration.winfo_children():
            try:
                if child.cget('text') == '模型显示校正（仅画面）':
                    child.configure(text='左侧 U-Arm 模型校正')
            except tk.TclError:
                pass
        self.image_label = tk.Label(self.root, bg="black", borderwidth=0)
        self.image_label.pack(fill="both", expand=True)
        self._drag = None
        self.image_label.bind('<ButtonPress-1>', self._drag_start)
        self.image_label.bind('<B1-Motion>', self._drag_move)
        self.image_label.bind('<MouseWheel>', self._zoom)
        self.photo = None
        self.root.update_idletasks()
        self.root.deiconify()
        if hasattr(ctypes, "windll"):
            hwnd = ctypes.windll.user32.FindWindowW(None, window_title)
            if hwnd:
                dpi = int(ctypes.windll.user32.GetDpiForWindow(hwnd)) or 96
                self.dpi_scale = dpi / 96.0
                # This thread is already PER_MONITOR_AWARE_V2, so MoveWindow
                # consumes physical pixels.  Multiplying by dpi/96 here makes
                # a 1200 px window 2100 px wide on a 175% display and crops it.
                ctypes.windll.user32.MoveWindow(
                    hwnd,
                    30,
                    30,
                    1200,
                    850,
                    True,
                )
                self.root.update_idletasks()

    def _camera_at(self, x):
        return self.uarm_camera if x < self.image_label.winfo_width() / 2 else self.cr5_camera

    def consume_sim_actions(self):
        actions = self.sim_actions[:]
        self.sim_actions.clear()
        return actions

    def _sync_sim_reverse(self):
        index = self.inspect_axis.get()-1
        self.sim_reverse.set(self.sim_signs[index] < 0)
        self.sim_scale.set(f'{self.sim_scales[index]:g}')
        self.sim_target_deg.set(f'{self.latest_virtual_joints[index]:.2f}')

    def _change_sim_reverse(self):
        index = self.inspect_axis.get()-1
        value = -1.0 if self.sim_reverse.get() else 1.0
        self.sim_signs[index] = value
        self.sim_actions.append(('sign',index,value))
        self.sim_saved_text.set(f'J{index+1} 已修改，待保存；仅右侧仿真')

    def _apply_sim_scale(self):
        index = self.inspect_axis.get()-1
        try:
            value = float(self.sim_scale.get())
        except ValueError:
            self.sim_saved_text.set('倍率不是有效数字')
            return
        if not math.isfinite(value) or not 0.1 <= value <= 2.0:
            self.sim_saved_text.set('仿真倍率必须在 0.1 到 2.0 之间')
            return
        self.sim_actions.append(('scale',index,value))
        self.sim_saved_text.set(f'J{index+1} 倍率待应用；仅右侧仿真')

    def _fill_actual_target(self):
        index = self.inspect_axis.get()-1
        self.sim_target_deg.set(f'{self.latest_actual_joints[index]:.2f}')

    def _apply_virtual_target(self):
        index = self.inspect_axis.get()-1
        try:
            value = float(self.sim_target_deg.get())
        except ValueError:
            self.sim_saved_text.set('虚拟目标角不是有效数字')
            return
        if not math.isfinite(value):
            self.sim_saved_text.set('虚拟目标角必须是有限数字')
            return
        self.sim_actions.append(('manual_target',index,value))
        self.sim_saved_text.set(f'J{index+1} 虚拟目标待应用；不控制真机')

    def set_joint_values(self, actual: list[float], virtual: list[float]) -> None:
        self.latest_actual_joints = list(actual)
        self.latest_virtual_joints = list(virtual)

    def _drag_start(self, event):
        self._drag = (event.x, event.y, self._camera_at(event.x))

    def _drag_move(self, event):
        if self._drag:
            x, y, camera = self._drag
            camera.azimuth -= (event.x-x) * .4
            camera.elevation = clamp(camera.elevation + (event.y-y)*.3, -85, -3)
            self._drag = (event.x, event.y, camera)

    def _zoom(self, event):
        camera = self._camera_at(event.x)
        camera.distance = clamp(camera.distance * (.9 if event.delta > 0 else 1.1), .2, 4.0)

    def _request_close(self) -> None:
        self.closed = True

    def _toggle_fullscreen(self, _event=None) -> None:
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def _leave_fullscreen(self, _event=None) -> None:
        if self.fullscreen:
            self.fullscreen = False
            self.root.attributes("-fullscreen", False)

    def is_running(self) -> bool:
        return not self.closed

    def update(
        self, data: mujoco.MjData, left_text: str, right_text: str
    ) -> None:
        if self.closed:
            return
        self.renderer.update_scene(
            data, camera=self.uarm_camera, scene_option=self.uarm_option
        )
        self._draw_axis(data, 'u_', .07)
        left_rgb = self.renderer.render().copy()
        self.renderer.update_scene(
            data, camera=self.cr5_camera, scene_option=self.cr5_option
        )
        self._draw_axis(data, 'a_', .18)
        right_rgb = self.renderer.render().copy()
        import numpy as np
        rgb = np.concatenate((left_rgb, right_rgb), axis=1)
        image = self.Image.fromarray(rgb)
        self.photo = self.ImageTk.PhotoImage(image=image)
        self.image_label.configure(image=self.photo)
        self.status.configure(text=f"{left_text}\n{right_text}")
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.closed = True

    def _draw_axis(self, data, prefix, length):
        if not self.highlight_axis.get():
            return
        import numpy as np
        index = self.inspect_axis.get()
        jid = self.model.joint(f'{prefix}joint{index}').id
        point, axis = data.xanchor[jid], data.xaxis[jid]
        scene = self.renderer.scene
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),
                           point,np.eye(3).flatten(),np.array([1.,.9,.05,1.],dtype=np.float32))
        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,length*.06,
                            point-axis*length*.25,point+axis*length*.75)
        geom.label = f'J{index}'
        scene.ngeom += 1

    def close(self) -> None:
        self.closed = True
        self.renderer.close()
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


class DMStatusPoller:
    def __init__(self, host: str, config: dict[str, Any]) -> None:
        self.host = host
        self.config = config
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest: dict[str, Any] | None = None
        self.error: str | None = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                value = read_dm_preflight(self.host, self.config)
                with self.lock:
                    self.latest = value
                    self.error = None
            except Exception as exc:
                with self.lock:
                    self.error = f"{type(exc).__name__}: {exc}"
            self.stop_event.wait(0.5)

    def snapshot(self) -> tuple[dict[str, Any] | None, str | None]:
        with self.lock:
            return (dict(self.latest) if self.latest is not None else None, self.error)

    def close(self) -> None:
        self.stop_event.set()
        # One bounded status call can take 3 s to connect and 5 s to receive.
        self.thread.join(timeout=9.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.201.1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--duration", type=float, default=900.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument('--resume-simulation', type=Path,
                        help='local twin log whose last virtual pose is retained; simulation only')
    parser.add_argument(
        "--geometry-preview",
        action="store_true",
        help="offline STEP/URDF geometry view; opens no serial or network connection",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.rate < 5 or args.rate > 50:
        parser.error("duration must be positive and rate must be within 5..50 Hz")

    calibration = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    health_config = json.loads(HEALTH_CONFIG.read_text(encoding="utf-8"))
    gripper_config = json.loads(UARM_GRIPPER_CONFIG.read_text(encoding="utf-8"))
    dm_config = json.loads(DM_CONFIG.read_text(encoding="utf-8"))
    sim_config = json.loads(SIM_CONFIG.read_text(encoding='utf-8'))
    if sim_config.get('purpose') != 'simulation_only':
        raise ValueError('simulation mapping requires simulation_only configuration')
    sim_signs = sim_config['joint_signs']
    sim_scales = sim_config.get('joint_scales', [1.0] * 6)
    sim_offsets = [float(value) for value in sim_config.get('joint_offsets_deg', [0.0] * 6)]
    if len(sim_offsets) != 6 or not all(math.isfinite(value) for value in sim_offsets):
        raise ValueError('joint_offsets_deg must contain six finite numbers')
    resume_sample = None
    if args.resume_simulation:
        with args.resume_simulation.open('rb') as stream:
            stream.seek(0,2)
            stream.seek(max(0,stream.tell()-100000))
            for line in stream.read().splitlines():
                try:
                    candidate = json.loads(line)
                    if candidate.get('kind')=='sample':
                        resume_sample = candidate
                except (ValueError, UnicodeDecodeError):
                    continue
        if resume_sample is None:
            raise ValueError('No simulation sample in resume log')
    center = [int(value) for value in calibration["leader"]["center_candidate_ticks"]]
    signs = [float(value) for value in calibration["mapping"]["joint_signs"]]
    candidate_home = [float(value) for value in calibration["robot"]["safe_home_candidate"]["joint_deg_canonical_equivalent"]]
    if len(center) != 6 or len(signs) != 6 or len(candidate_home) != 6:
        raise ValueError("incomplete six-axis calibration")

    uarm_open = int(gripper_config["endpoints"]["maximum_open"]["ticks"])
    uarm_close = int(gripper_config["endpoints"]["fully_closed"]["ticks"])
    dm_open = float(gripper_config["mapping"]["dm4310_target_range"]["safe_open_command_rad"])
    dm_close = float(gripper_config["mapping"]["dm4310_target_range"]["safe_close_command_rad"])

    model, data = build_scene()
    model_limits = [list(map(math.degrees, model.jnt_range[model.joint(f'a_joint{i}').id]))
                    for i in range(1, 7)]
    sim_follow = SimFollowMapping(signs=sim_signs, scales=sim_scales,
                                  limits_deg=model_limits)
    sim_initialized = False
    sim_paused = False
    held_target = None
    held_gripper = 0.0
    cr5_geom_ids = [i for i in range(model.ngeom) if model.geom_group[i] == 2]
    if args.geometry_preview:
        if not args.gui:
            parser.error("--geometry-preview requires --gui")
        preview_gui: TwinCanvas | None = None
        try:
            mujoco.mj_forward(model, data)
            preview_gui = TwinCanvas(
                model,
                window_title="U-Arm Config2 corrected STEP view FINAL (OFFLINE)",
                input_reference={"model": "uarm_output_axes_v1", "source": "CAD_zero_offline"},
            )
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline and preview_gui.is_running():
                display = preview_gui.registration.map_angles([0.0]*7)
                for i in range(7):
                    set_joint_rad(model, data, f'u_joint{i+1}' if i<6 else 'u_trigger_joint', math.radians(display[i]))
                mujoco.mj_forward(model, data)
                preview_gui.update(
                    data,
                    "LEFT: U-Arm Config2 / official STEP geometry / CAD q=0",
                    (
                        "RIGHT: CR5AF model reference\n"
                        "OFFLINE GEOMETRY AUDIT: no COM / TCP / hardware access"
                    ),
                )
                time.sleep(0.05)
        finally:
            if preview_gui is not None:
                preview_gui.close()
        return 0

    port_name = resolve_uarm_port(args.port)
    port = PortHandler(port_name)
    protocol = sms_sts(port)
    feedback_stream: FeedbackStream | None = None
    opened = False
    gui: TwinCanvas | None = None
    dm_poller: DMStatusPoller | None = None
    started = time.monotonic()
    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"uarm_cr5_dual_twin_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    sample_number = 0
    try:
        opened = bool(port.openPort())
        if not opened or not port.setBaudRate(1_000_000):
            raise RuntimeError(f"could not open {port_name} at 1 Mbps")
        health = read_servo_health(protocol)
        require_uarm_health(health, health_config)
        positions, transport = read_positions(protocol)
        if any(
            int(item["comm_result"]) != COMM_SUCCESS
            or int(item["packet_error"]) != 0
            for item in transport
        ):
            raise RuntimeError(f"initial U-Arm read failed: {transport}")
        previous = positions[:]
        unwrapped = [shortest_tick_step(center[i], positions[i]) for i in range(6)]

        feedback_stream = FeedbackStream(args.host, timeout=3.0)
        feedback_stream.wait_first(timeout=3.0)
        feedback_reconnects = 0
        dm_poller = DMStatusPoller(args.host, dm_config)
        dm_poller.start()

        if args.gui:
            gui = TwinCanvas(model, input_reference={
                "model": "uarm_output_axes_v1", "center_ticks": center + [uarm_close],
            }, sim_controls=True, simulation_signs=sim_signs,
               simulation_scales=sim_scales)

        with log_path.open("x", encoding="utf-8") as log:
            session = {
                "timestamp_utc": utc_now(),
                "kind": "session",
                "scene": "left live U-Arm; right simulation follower by default, actual feedback selectable",
                "uarm_port": port_name,
                "leader_center_ticks": center,
                "simulation_gain": 1.0,
                "simulation_limits_deg": model_limits,
                "simulation_signs": sim_signs,
                "simulation_scales": sim_scales,
                "simulation_joint_offsets_deg": sim_offsets,
                "simulation_reference_policy": "pair current leader with measured robot at startup; changes are session-only",
                "simulation_hardware_motion_allowed": False,
                "uarm_display_home_deg": UARM_DISPLAY_HOME_DEG,
                "uarm_display_scale": UARM_DISPLAY_SCALE,
                "uarm_display_home_note": (
                    "exact STEP assembly pose; live deltas only; physical absolute pose not registered"
                ),
                "health_warnings": uarm_health_warnings(health, health_config),
                "commands_to_cr5": 0,
                "writes_to_uarm": 0,
                "gripper_action_commands": 0,
                "feedback_policy": "background receive-only drain; stop on invalid or stale feedback; never send",
            }
            log.write(json.dumps(session, ensure_ascii=False) + "\n")
            deadline = time.monotonic() + args.duration
            next_tick = time.monotonic()
            period = 1.0 / args.rate
            while time.monotonic() < deadline and (gui is None or gui.is_running()):
                positions, transport = read_positions(protocol)
                if any(
                    int(item["comm_result"]) != COMM_SUCCESS
                    or int(item["packet_error"]) != 0
                    for item in transport
                ):
                    raise RuntimeError(f"U-Arm runtime read failed: {transport}")
                for index in range(6):
                    unwrapped[index] += shortest_tick_step(previous[index], positions[index])
                previous = positions
                servo_relative_deg = [
                    unwrapped[i] * DEGREES_PER_TICK for i in range(6)
                ]
                trigger_relative = shortest_tick_step(uarm_close, int(positions[6])) * DEGREES_PER_TICK
                display_input = servo_relative_deg + [trigger_relative]
                display_angles = (gui.registration.map_angles(display_input) if gui is not None
                                  else DisplayRegistration().map_angles(display_input))
                mapped_relative_deg = [
                    signs[i] * servo_relative_deg[i] for i in range(6)
                ]

                feedback = feedback_stream.latest(maximum_age=0.25)
                actual_joints = [float(value) for value in feedback["q_actual_deg"]]
                model_feedback_joints = [actual_joints[i] + sim_offsets[i] for i in range(6)]
                rebase_reason = None
                if not sim_initialized:
                    if resume_sample is None:
                        sim_follow.rebase(positions[:6], model_feedback_joints, reason='startup_actual_mapped')
                        rebase_reason = 'startup_actual'
                    else:
                        sim_follow.rebase(resume_sample['uarm_ticks'][:6],
                                          resume_sample['cr5_preview_joint_deg'],reason='resume_display')
                        rebase_reason = 'resume_display'
                    sim_initialized = True
                for action in (gui.consume_sim_actions() if gui is not None else []):
                    if isinstance(action,tuple) and action[0]=='sign':
                        _, axis_index, direction = action
                        updated_signs = list(sim_signs)
                        updated_signs[axis_index] = direction
                        updated_mapping = SimFollowMapping(signs=updated_signs,
                                                           scales=sim_scales,
                                                           limits_deg=model_limits)
                        updated_mapping.rebase(positions[:6],held_target or model_feedback_joints,
                                               reason=f'direction_J{axis_index+1}')
                        sim_follow, sim_signs = updated_mapping, updated_signs
                        gui.sim_mode.set('simulation')
                        rebase_reason = f'direction_J{axis_index+1}'
                    elif isinstance(action,tuple) and action[0]=='scale':
                        _, axis_index, scale = action
                        updated_scales = list(sim_scales)
                        updated_scales[axis_index] = float(scale)
                        updated_mapping = SimFollowMapping(signs=sim_signs,
                                                           scales=updated_scales,
                                                           limits_deg=model_limits)
                        updated_mapping.rebase(positions[:6],held_target or model_feedback_joints,
                                               reason=f'scale_J{axis_index+1}')
                        sim_follow, sim_scales = updated_mapping, updated_scales
                        gui.sim_scales = list(sim_scales)
                        gui.sim_scale.set(f'{scale:g}')
                        gui.sim_mode.set('simulation')
                        rebase_reason = f'scale_J{axis_index+1}'
                    elif isinstance(action,tuple) and action[0]=='manual_target':
                        _, axis_index, value = action
                        lower, upper = model_limits[axis_index]
                        if not math.isfinite(float(value)) or not lower <= float(value) <= upper:
                            gui.sim_saved_text.set(
                                f'J{axis_index+1} 目标超出模型范围 {lower:.1f}°..{upper:.1f}°')
                        else:
                            candidate = list(held_target or model_feedback_joints)
                            candidate[axis_index] = float(value)
                            updated_mapping = SimFollowMapping(signs=sim_signs,
                                                               scales=sim_scales,
                                                               limits_deg=model_limits)
                            updated_mapping.rebase(positions[:6],candidate,
                                                   reason=f'manual_target_J{axis_index+1}')
                            sim_follow = updated_mapping
                            held_target = candidate
                            sim_paused = False
                            gui.pause_text.set('暂停仿真')
                            gui.sim_mode.set('simulation')
                            rebase_reason = f'manual_target_J{axis_index+1}'
                            gui.sim_saved_text.set(
                                f'J{axis_index+1} 虚拟目标已应用；只影响仿真')
                    elif action == 'save_signs':
                        try:
                            payload = dict(sim_config)
                            payload.update({'schema_version':1,'purpose':'simulation_only',
                                            'joint_signs':list(sim_signs),
                                            'joint_scales':list(sim_scales),
                                            'joint_offsets_deg':list(sim_offsets),
                                            'saved_at_utc':utc_now(),
                                            'note':'Simulation-only joint direction, scale, and real-feedback-to-model pose offsets; live teleop mapping and robot controller settings unchanged.'})
                            temporary = SIM_CONFIG.with_suffix('.tmp')
                            temporary.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
                            temporary.replace(SIM_CONFIG)
                            gui.sim_saved_text.set('仿真方向已保存；实机映射未改')
                        except OSError as exc:
                            gui.sim_saved_text.set(f'保存失败：{exc}')
                    elif action in ('actual', 'candidate'):
                        anchor = model_feedback_joints if action == 'actual' else [candidate_home[i] + sim_offsets[i] for i in range(6)]
                        sim_follow.rebase(positions[:6], anchor,
                                          reason=action)
                        held_target = anchor[:]
                        sim_paused = False
                        gui.pause_text.set('暂停仿真')
                        gui.sim_mode.set('simulation')
                        rebase_reason = action
                    elif action == 'pause':
                        if sim_paused:
                            sim_follow.rebase(positions[:6], held_target, reason='resume_hold')
                            rebase_reason = 'resume_hold'
                        sim_paused = not sim_paused
                        gui.pause_text.set('继续仿真' if sim_paused else '暂停仿真')
                sim_result = sim_follow.update(positions[:6])
                if not sim_paused or held_target is None:
                    held_target = sim_result['target_deg'][:]
                preview_joints = held_target[:]
                show_simulation = gui is None or gui.sim_mode.get() == 'simulation'
                shown_joints = preview_joints if show_simulation else model_feedback_joints
                if rebase_reason:
                    log.write(json.dumps({'kind':'simulation_reference','timestamp_utc':utc_now(),
                                          'reference':sim_follow.as_dict()},ensure_ascii=False)+'\n')
                    log.flush()

                for index in range(6):
                    set_joint_rad(
                        model, data, f"u_joint{index + 1}",
                        math.radians(display_angles[index]),
                    )
                    set_joint_rad(
                        model, data, f"a_joint{index + 1}",
                        math.radians(shown_joints[index]),
                    )
                set_joint_rad(model, data, 'u_trigger_joint', math.radians(display_angles[6]))

                uarm_open_fraction = clamp(
                    (uarm_close - int(positions[6])) / (uarm_close - uarm_open),
                    0.0, 1.0,
                )
                dm_feedback, dm_error = dm_poller.snapshot()
                if dm_feedback is None:
                    dm_open_fraction = 0.0
                else:
                    dm_open_fraction = clamp(
                        (dm_close - float(dm_feedback["position_rad"]))
                        / (dm_close - dm_open), 0.0, 1.0
                    )
                if not sim_paused:
                    held_gripper = uarm_open_fraction
                set_tool_open_fraction(model, data, "a_", held_gripper if show_simulation else dm_open_fraction)
                model.geom_rgba[cr5_geom_ids] = ([0.15,0.85,0.35,1.0] if show_simulation
                                               else [0.92,0.46,0.10,1.0])

                mujoco.mj_forward(model, data)
                if gui is not None:
                    gui.set_joint_values(model_feedback_joints, preview_joints)
                    inspected = gui.inspect_axis.get()-1
                    gui.inspection_text.set(
                        f"输入 {positions[inspected]} ticks | 虚拟 J{inspected+1} {preview_joints[inspected]:+.1f}°"
                        f" | 实机反馈映射 {model_feedback_joints[inspected]:+.1f}°")
                    status_left = (
                        "左：U-Arm 实时输入 | 拖动旋转视角，滚轮缩放"
                    )
                    status_right = (
                        f"右：{'绿色虚拟 CR5 跟随' if show_simulation else '橙色 CR5 实机反馈映射'}"
                        f" | {('仿真已暂停' if sim_paused else '按逐轴倍率跟随') if show_simulation else '只读反馈，经模型坐标偏置映射'}"
                        f" | 模型限位：{sim_result['clipped_axes'] or '未触及'}\n"
                        f"实机：模式 {feedback['robot_mode']} / 使能 {feedback['enable_status']} / 报警 {feedback['error_status']}"
                        " | 本窗口不发送动作指令\n"
                        f"仿真角度：{'  '.join(f'{v:+.1f}' for v in preview_joints)}°"
                        f" | 实机映射：{'  '.join(f'{v:+.1f}' for v in model_feedback_joints)}°"
                        f" | {'夹爪反馈不可用' if dm_error or dm_feedback is None else '夹爪状态已读取'}"
                        " | 范围与碰撞尚未实机验证"
                    )
                    gui.update(data, status_left, status_right)

                record = {
                    "timestamp_utc": utc_now(),
                    "kind": "sample",
                    "sample": sample_number,
                    "uarm_ticks": positions,
                    "leader_servo_relative_deg": servo_relative_deg,
                    "uarm_display_angles_deg": display_angles,
                    "mapped_relative_deg": mapped_relative_deg,
                    "uarm_gripper_open_fraction": uarm_open_fraction,
                    "cr5_actual_joint_deg": actual_joints,
                    "cr5_model_feedback_joint_deg": model_feedback_joints,
                    "cr5_feedback_model_offsets_deg": sim_offsets,
                    "cr5_preview_joint_deg": preview_joints,
                    "cr5_rendered_joint_deg": shown_joints,
                    "cr5_render_source": 'simulation' if show_simulation else 'actual_feedback',
                    "simulation_paused": sim_paused,
                    "simulation_delta_deg": sim_result['delta_deg'],
                    "simulation_clipped_axes": sim_result['clipped_axes'],
                    "simulation_reference": sim_follow.as_dict(),
                    "simulation_gripper_open_fraction": held_gripper,
                    "cr5_robot_mode": feedback["robot_mode"],
                    "cr5_enable_status": feedback["enable_status"],
                    "cr5_error_status": feedback["error_status"],
                    "cr5_controller_timestamp_ms": feedback["timestamp_ms"],
                    "cr5_feedback_fresh_verified": False,
                    "feedback_reconnects": feedback_reconnects,
                    "dm_feedback": dm_feedback,
                    "dm_poll_error": dm_error,
                    "dm_open_fraction": dm_open_fraction,
                }
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                if sample_number % max(1, int(args.rate)) == 0:
                    log.flush()
                sample_number += 1
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
    finally:
        try:
            if gui is not None:
                gui.close()
        finally:
            try:
                if dm_poller is not None:
                    dm_poller.close()
            finally:
                try:
                    if feedback_stream is not None:
                        feedback_stream.close()
                finally:
                    if opened and port.ser is not None:
                        port.closePort()

    print(json.dumps({
        "log": str(log_path),
        "samples": sample_number,
        "commands_to_cr5": 0,
        "writes_to_uarm": 0,
        "gripper_action_commands": 0,
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
