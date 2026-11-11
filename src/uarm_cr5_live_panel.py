"""Local start/stop panel for continuous physical teleoperation.

Opening this panel does not open hardware. Only the Start button launches the
controller. The child owns all hardware; the panel writes a heartbeat and stop
file and reads telemetry. Closing requests a stop and waits for child cleanup.
"""
from __future__ import annotations

import argparse
import ctypes
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
CONFIRM = "I_UNDERSTAND_CONTINUOUS_CR5_WILL_MOVE"
RECOVERY_CONFIRM = "I_UNDERSTAND_CR5_WILL_RETURN_HOME"
VIEWER_TITLE = "U-Arm / CR5 实时联动监视（文件读取）"


def tail_rows(path: Path | None) -> list[dict]:
    if path is None or not path.exists():
        return []
    with path.open("rb") as stream:
        stream.seek(0, 2)
        stream.seek(max(0, stream.tell() - 24000))
        lines = stream.read().splitlines()
    result = []
    for line in lines:
        try:
            result.append(json.loads(line))
        except (ValueError, UnicodeDecodeError):
            pass
    return result


class LivePanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("U-Arm → CR5 真机连续联动")
        root.geometry("1080x850+40+40")
        root.minsize(960, 720)
        self.process = None
        self.recovery_process = None
        self.console = None
        self.stream = self.heartbeat = self.stop_path = None
        self.stop_requested_at = None
        self.stop_mode = None
        self.closing = False
        self.uncertain_stop = False
        self.editable = []
        self.status = tk.StringVar(value="待启动：真实 CR5 六轴与夹爪均响应 U-Arm。")
        self.detail = tk.StringVar(value="当前没有控制程序运行。")
        self.joint_readout = tk.StringVar(value="尚无本轮实机反馈")
        self.last_error = tk.StringVar(value="")
        self.arm_only = tk.BooleanVar(value=False)
        config = json.loads((ROOT / "config/uarm_dobot_teleop.json").read_text(encoding="utf-8"))
        self.default_signs = list(config["joint_signs"])
        self.enabled = [tk.BooleanVar(value=True) for _ in range(6)]
        self.reversed = [tk.BooleanVar(value=False) for _ in range(6)]
        self.sources = [tk.StringVar(value=str(i+1)) for i in range(6)]
        self.scales = [tk.StringVar(value="1.0") for _ in range(6)]
        self.offsets = [tk.StringVar(value="0") for _ in range(6)]
        self.max_speed = tk.StringVar(value="12")
        self.max_follow_lag = tk.StringVar(value="20")
        preferences = ROOT / "config/uarm_cr5_continuous_ui.json"
        if preferences.exists():
            saved = json.loads(preferences.read_text(encoding="utf-8"))
            signs = saved["joint_signs"]
            selected = saved["joints"]
            sources = saved.get("joint_sources", list(range(1,7)))
            scales = saved.get("joint_scales", [1.0]*6)
            offsets = saved.get("joint_offsets_deg", [0.0]*6)
            max_speed = float(saved.get("max_speed_deg_s", 12.0))
            max_follow_lag = float(saved.get("max_follow_lag_deg", 20.0))
            if (len(signs) != 6 or any(s not in (-1, 1) for s in signs)
                    or not selected or not set(selected) <= set(range(1, 7))
                    or sorted(sources) != list(range(1,7))
                    or len(scales) != 6 or any(not 0.1 <= float(v) <= 2.0 for v in scales)
                    or len(offsets) != 6 or any(not -180.0 <= float(v) <= 180.0 for v in offsets)
                    or not 1.0 <= max_speed <= 20.0
                    or not 0.0 <= max_follow_lag <= 180.0):
                raise ValueError("Invalid saved continuous-control directions/selection")
            for i in range(6):
                self.reversed[i].set(signs[i] != self.default_signs[i])
                self.enabled[i].set(i + 1 in selected)
                self.sources[i].set(str(sources[i]))
                self.scales[i].set(f"{float(scales[i]):g}")
                self.offsets[i].set(f"{float(offsets[i]):g}")
            self.max_speed.set(f"{max_speed:g}")
            self.max_follow_lag.set(f"{max_follow_lag:g}")
        body = ttk.Frame(root, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="真机连续联动", font=("Microsoft YaHei UI", 21, "bold")).pack(anchor="w")
        ttk.Label(body, text="持续运行 · 手动停止 · 不再限制为每轴 ±5°", font=("Microsoft YaHei UI", 12)).pack(anchor="w", pady=(4, 12))
        arm_only_box = ttk.Checkbutton(
            body,
            text="本轮仅控制 CR5 机械臂；跳过 DM 网关，不发送夹爪命令",
            variable=self.arm_only,
        )
        arm_only_box.pack(anchor="w", pady=(0, 6))
        self.editable.append(arm_only_box)
        ttk.Label(body, textvariable=self.status, font=("Microsoft YaHei UI", 12, "bold"), wraplength=860).pack(anchor="w", pady=5)
        ttk.Label(body, textvariable=self.detail, wraplength=860).pack(anchor="w", pady=5)
        frame = ttk.LabelFrame(body, text="关节映射（只能在停止后调整）", padding=10)
        frame.pack(fill="x", pady=12)
        for i in range(6):
            cell = ttk.Frame(frame, padding=5)
            cell.grid(row=0, column=i, sticky="nsew")
            frame.columnconfigure(i, weight=1)
            cb = ttk.Checkbutton(cell, text=f"J{i + 1} 跟随", variable=self.enabled[i])
            cb.pack(anchor="w")
            ttk.Label(cell, text="来源 U-Arm ID").pack(anchor="w", pady=(6,0))
            source = ttk.Combobox(cell, textvariable=self.sources[i],
                                  values=("1","2","3","4","5","6"), width=5,
                                  state="readonly")
            source.pack(anchor="w")
            rev = ttk.Checkbutton(cell, text="反向", variable=self.reversed[i])
            rev.pack(anchor="w", pady=6)
            ttk.Label(cell, text="角度倍率").pack(anchor="w")
            scale = ttk.Combobox(cell, textvariable=self.scales[i],
                                 values=("0.25","0.5","0.75","1","1.25","1.5","2"),
                                 width=5)
            scale.pack(anchor="w")
            ttk.Label(cell, text="待应用偏置°").pack(anchor="w", pady=(5,0))
            offset = ttk.Combobox(cell, textvariable=self.offsets[i],
                                  values=("-180","-90","0","90","180"),
                                  width=5)
            offset.pack(anchor="w")
            ttk.Label(cell, text=f"原方向 {int(self.default_signs[i]):+d}").pack(anchor="w")
            self.editable.extend([cb, source, rev, scale, offset])
        speed_row = ttk.Frame(body)
        speed_row.pack(fill="x", pady=(0,8))
        ttk.Label(speed_row, text="CR5 最大跟随速度：").pack(side="left")
        speed = ttk.Combobox(speed_row, textvariable=self.max_speed,
                             values=("3","6","10","12","15","20"), width=6)
        speed.pack(side="left")
        ttk.Label(speed_row, text="°/秒（推荐 12；原来是 3）").pack(side="left", padx=8)
        ttk.Label(speed_row, text="最大追赶差：").pack(side="left", padx=(25,0))
        follow_lag = ttk.Combobox(speed_row, textvariable=self.max_follow_lag,
                                  values=("10","20","30","60","0"), width=6)
        follow_lag.pack(side="left")
        ttk.Label(speed_row, text="°（推荐 20；0=不限制）").pack(side="left", padx=8)
        self.editable.append(speed)
        self.editable.append(follow_lag)
        ttk.Label(body, text="启动时配对两臂当前位置；每个 CR5 关节按所选 U-Arm ID、方向、倍率和偏置跟随。仅机械臂模式不会访问 DM 网关或控制夹爪。", wraplength=1000).pack(anchor="w")
        ttk.Label(body, text="夹爪扳机为全开/全闭两态：中间区维持上次目标，不能停在任意开度。夹住物体后低力保持；反向松开期间可再次扣动夹紧。", wraplength=1000).pack(anchor="w")
        ttk.Label(body, text="偏置按最大跟随速度逐步加入；完成或安全停止后自动扣除已执行角度，避免下次重复叠加。180°在12°/秒时至少需要15秒。", foreground="#a34100", wraplength=1000).pack(anchor="w", pady=3)
        ttk.Label(body, text="保留关节限位。反向拨动使目标离开限位，机械臂按所选限速跟随。", wraplength=1000).pack(anchor="w", pady=5)
        ttk.Label(body, text="主臂大幅快拨后，CR5 仍会继续走向目标；按停止会取消尚未完成的目标。", wraplength=900).pack(anchor="w", pady=5)
        ttk.Label(body, text="大幅运动需要现场避开桌面、人体、线缆和自身碰撞；程序不判断周围障碍物。", foreground="#a34100", wraplength=860).pack(anchor="w", pady=5)
        actions = ttk.Frame(body)
        actions.pack(fill="x", pady=15)
        self.start_button = tk.Button(actions, text="开始真机联动（持续）", font=("Microsoft YaHei UI", 14, "bold"), bg="#167a42", fg="white", command=self.start, padx=15, pady=12)
        self.start_button.pack(side="left", fill="x", expand=True, padx=(0, 10))
        self.return_button = tk.Button(actions, text="主动回安全零位后停止", font=("Microsoft YaHei UI", 14, "bold"), bg="#b36b00", fg="white", command=self.return_home, padx=15, pady=12, state="disabled")
        self.return_button.pack(side="left", fill="x", expand=True, padx=(0, 10))
        self.stop_button = tk.Button(actions, text="原地停止并失能（不回零）", font=("Microsoft YaHei UI", 14, "bold"), bg="#b82727", fg="white", command=self.emergency_stop, padx=15, pady=12, state="disabled")
        self.stop_button.pack(side="left", fill="x", expand=True)
        ttk.Label(body, text="红色按钮和关闭窗口均原地停止、不回零；只有主动点击橙色按钮才回安全零位。软件停止不代替现场实体急停。", foreground="#a12626", wraplength=1000).pack(anchor="w")
        ttk.Label(body, textvariable=self.joint_readout, font=("Consolas", 11), wraplength=900).pack(anchor="w", pady=12)
        ttk.Label(body, textvariable=self.last_error, foreground="#b82727", wraplength=860).pack(anchor="w")
        tools_row = ttk.Frame(body)
        tools_row.pack(side="bottom", fill="x", pady=8)
        ttk.Button(tools_row, text="打开 3D 实测监视", command=self.open_viewer).pack(side="left")
        ttk.Button(tools_row, text="打开本轮日志", command=self.open_logs).pack(side="left", padx=10)
        self.recovery_button = ttk.Button(
            tools_row,
            text="碰撞清警并回安全零位",
            command=self.recover_collision,
        )
        self.recovery_button.pack(side="right")
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.tick)

    def write_heartbeat(self):
        # mtime and content both carry freshness for the controller.
        self.heartbeat.write_text(str(time.time()), encoding="ascii")

    def start(self):
        if self.process is not None or self.recovery_process is not None or self.uncertain_stop:
            return
        selected = [str(i + 1) for i, var in enumerate(self.enabled) if var.get()]
        if not selected:
            self.last_error.set("请至少选择一个关节。")
            return
        try:
            sources = [int(var.get()) for var in self.sources]
            scales = [float(var.get()) for var in self.scales]
            offsets = [float(var.get()) for var in self.offsets]
            max_speed = float(self.max_speed.get())
            max_follow_lag = float(self.max_follow_lag.get())
        except ValueError:
            self.last_error.set("来源 ID、角度倍率、偏置或最大速度不是有效数字。")
            return
        if sorted(sources) != list(range(1,7)):
            self.last_error.set("六个来源必须各使用 U-Arm ID1–6 一次，不能重复或遗漏。")
            return
        if any(not 0.1 <= value <= 2.0 for value in scales):
            self.last_error.set("每轴角度倍率必须在 0.1–2.0。")
            return
        if any(not -180.0 <= value <= 180.0 for value in offsets):
            self.last_error.set("每轴角度偏置必须在 -180°到 +180°。")
            return
        if not 1.0 <= max_speed <= 20.0:
            self.last_error.set("CR5 最大跟随速度必须在 1–20°/秒。")
            return
        if not 0.0 <= max_follow_lag <= 180.0:
            self.last_error.set("最大追赶差必须在 0–180°；0 表示不限制。")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_dir = ROOT / "logs" / f"continuous_session_{stamp}"
        run_dir.mkdir()
        self.heartbeat = run_dir / "gui.heartbeat"
        self.stop_path = run_dir / "stop.request"
        self.stream = ROOT / "logs" / f"uarm_cr5_control_stream_{stamp}.jsonl"
        signs = [int(self.default_signs[i]) * (-1 if self.reversed[i].get() else 1) for i in range(6)]
        try:
            preferences = ROOT / "config/uarm_cr5_continuous_ui.json"
            pending = preferences.with_suffix(".tmp")
            pending.write_text(json.dumps({
                "joint_signs": signs,
                "joint_sources": sources,
                "joint_scales": scales,
                "joint_offsets_deg": offsets,
                "max_speed_deg_s": max_speed,
                "max_follow_lag_deg": max_follow_lag,
                "joints": [int(j) for j in selected],
            }, indent=2), encoding="utf-8")
            pending.replace(preferences)
            self.write_heartbeat()
            self.console = (run_dir / "console.txt").open("w", encoding="utf-8")
            command = [
                sys.executable, "-u", str(ROOT / "src/uarm_cr5_continuous_teleop.py"),
                "--live-output", "--enable-robot", "--confirm", CONFIRM,
                "--stop-file", str(self.stop_path), "--heartbeat-file", str(self.heartbeat),
                "--telemetry-log", str(self.stream), "--joints", ",".join(selected),
                "--joint-signs=" + ",".join(str(s) for s in signs),
                "--joint-sources=" + ",".join(str(v) for v in sources),
                "--joint-scales=" + ",".join(f"{v:g}" for v in scales),
                "--joint-offsets-deg=" + ",".join(f"{v:g}" for v in offsets),
                "--max-speed-deg-s", f"{max_speed:g}",
                "--max-follow-lag-deg", f"{max_follow_lag:g}",
            ]
            if self.arm_only.get():
                command.append("--arm-only")
            self.process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=self.console, stderr=subprocess.STDOUT,
                env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        except Exception as exc:
            if self.console:
                self.console.close()
            self.console = None
            self.process = None
            self.last_error.set(f"启动失败：{exc}")
            return
        self.stop_requested_at = None
        self.stop_mode = None
        self.uncertain_stop = False
        self.last_error.set("")
        self.status.set("正在预检；通过后自动使能并开始真实跟随。" + ("（仅机械臂，夹爪不动作）" if self.arm_only.get() else ""))
        self.detail.set(
            f"本轮关节：{','.join(selected)}；来源：{sources}；方向：{signs}；"
            f"倍率：{scales}；偏置：{offsets}°；最高 {max_speed:g}°/秒；最大追赶差 {max_follow_lag:g}°。"
        )
        self.start_button.config(state="disabled")
        self.recovery_button.config(state="disabled")
        self.return_button.config(state="normal")
        self.stop_button.config(state="normal")
        for widget in self.editable:
            widget.config(state="disabled")

    def return_home(self):
        if self.process is None or self.stop_requested_at is not None:
            return
        self.stop_requested_at = time.monotonic()
        self.stop_mode = "return_home"
        self.status.set("正在返回已记录的操作归零位；到位后自动停止并失能。")
        self.return_button.config(state="disabled")
        try:
            self.stop_path.write_text("return_home\n", encoding="ascii")
        except OSError as exc:
            self.stop_mode = "emergency"
            self.last_error.set(f"归零请求写入失败：{exc}。请使用实体急停。")

    def emergency_stop(self):
        if self.process is None:
            return
        if self.stop_requested_at is None:
            self.stop_requested_at = time.monotonic()
        self.stop_mode = "emergency"
        self.status.set("正在立即停止并失能；不会自动回位。")
        self.return_button.config(state="disabled")
        try:
            self.stop_path.write_text("operator stop\n", encoding="ascii")
        except OSError as exc:
            self.last_error.set(f"停止文件写入失败：{exc}。请使用实体急停。")

    def tick(self):
        if self.recovery_process is not None:
            code = self.recovery_process.poll()
            if code is not None:
                output, _ = self.recovery_process.communicate()
                self.recovery_process = None
                try:
                    result = json.loads(output)
                except (TypeError, ValueError):
                    result = {}
                if code == 0 and result.get("passed"):
                    final = result.get("final", {})
                    self.status.set("碰撞已清除，已回到安全零位并确认失能。")
                    self.detail.set(
                        f"结束状态：模式 {final.get('robot_mode')}，使能 {final.get('enable_status')}，"
                        f"报警 {final.get('error_status')}。"
                    )
                    self.last_error.set("")
                    self.uncertain_stop = False
                    self.start_button.config(state="normal")
                else:
                    self.status.set("独立恢复未完成；机械臂不会自动继续运行。")
                    self.last_error.set(output.strip()[-1200:] or f"恢复程序退出码 {code}")
                self.recovery_button.config(state="normal")
        if self.process is not None:
            try:
                if self.stop_mode != "emergency":
                    self.write_heartbeat()
                rows = tail_rows(self.stream)
                sample = next((r for r in reversed(rows) if r.get("kind") in ("sample", "return_home")), None)
                terminal = next((r for r in reversed(rows) if r.get("kind") == "terminal"), None)
                if sample:
                    actual = sample["cr5"]
                    q = actual["q_actual_deg"]
                    text = "实测：" + "   ".join(f"J{i+1} {float(v):.1f}°" for i, v in enumerate(q))
                    target = sample.get("arm_target_deg") or sample.get("home_target_deg")
                    if target:
                        lag = max(abs(float(a) - float(b)) for a, b in zip(target, q))
                        text += "\n目标：" + "   ".join(f"J{i+1} {float(v):.1f}°" for i, v in enumerate(target))
                        text += f"\n目标与实测最大差：{lag:.1f}°"
                    offset_progress = sample.get("joint_offset_progress_deg")
                    offsets = sample.get("joint_offsets_deg")
                    if offset_progress and offsets:
                        active_offsets = [
                            f"J{i+1} {float(done):.1f}/{float(total):.1f}°"
                            for i, (done, total) in enumerate(zip(offset_progress, offsets))
                            if abs(float(total)) > .05
                        ]
                        if active_offsets:
                            text += "\n偏置进度：" + "   ".join(active_offsets)
                    if not self.arm_only.get() and sample.get("kind") == "sample":
                        names = {"open": "张开", "close": "夹紧"}
                        target_name = names.get(sample.get("gripper_desired"), "未指定")
                        if not sample.get("gripper_command_issued"):
                            gripper_state = "等待扳机跨过开/闭阈值"
                        elif sample.get("gripper_transition_pending"):
                            gripper_state = "正在转换方向/位置"
                        elif sample.get("gripper_contact_detected"):
                            gripper_state = "检测到接触，低力保持"
                        elif sample.get("gripper_holding_active"):
                            gripper_state = "端点保持"
                        else:
                            gripper_state = "等待网关确认"
                        ticks = sample.get("servo_ticks", [None] * 7)[6]
                        text += f"\n夹爪：目标{target_name}；{gripper_state}；扳机 ID7={ticks}"
                    self.joint_readout.set(text)
                    if self.stop_requested_at is None and not terminal:
                        age = (datetime.now(timezone.utc) - datetime.fromisoformat(sample["timestamp_utc"])).total_seconds()
                        if age > 1.0 or age < -1.0:
                            self.last_error.set("控制数据过期，已请求停止；若设备未停止，请使用实体急停。")
                            self.emergency_stop()
                        else:
                            self.status.set("真机正在跟随 U-Arm；" + ("夹爪已旁路。" if self.arm_only.get() else "扳机控制夹爪。"))
                    clipped = sample.get("limit_clipped_axes", [])
                    if clipped:
                        self.detail.set(f"关节 {clipped} 的目标已到软件限位；向反方向拨动使目标离开限位。")
                code = self.process.poll()
                if code is not None:
                    self.finish(code, terminal)
                    if self.closing and not self.uncertain_stop:
                        self.root.destroy()
                        return
                    self.closing = False
                elif self.stop_mode == "emergency" and time.monotonic() - self.stop_requested_at > 5:
                    self.last_error.set("停止确认仍在等待；若设备未停止，请立即使用实体急停。")
                elif self.stop_mode == "return_home" and time.monotonic() - self.stop_requested_at > 90:
                    self.last_error.set("归零超过 90 秒仍未完成，请点“立即停止并失能”或使用实体急停。")
            except Exception as exc:
                self.last_error.set(f"界面监视异常：{exc}；已请求停止。")
                self.emergency_stop()
        self.root.after(200, self.tick)

    def recover_collision(self):
        if self.process is not None:
            self.last_error.set("遥操仍在运行；先使用红色按钮原地停止并失能。")
            return
        if self.recovery_process is not None:
            return
        confirmed = messagebox.askyesno(
            "确认碰撞恢复条件",
            "仅在以下条件全部满足时继续：\n\n"
            "1. 人员、桌面、工件和线缆已离开机械臂回零路径；\n"
            "2. 碰撞接触已经解除；\n"
            "3. 实体急停已经释放并且随手可触及。\n\n"
            "确认现场已经清空并执行低速回零吗？",
            icon="warning",
        )
        if not confirmed:
            return
        command = [
            sys.executable,
            str(ROOT / "src/cr5_return_home.py"),
            "--execute",
            "--confirm",
            RECOVERY_CONFIRM,
            "--method",
            "movj",
            "--clear-collision",
        ]
        try:
            self.recovery_process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
        except Exception as exc:
            self.last_error.set(f"独立恢复程序启动失败：{exc}")
            return
        self.status.set("正在独立核验碰撞状态；通过后清警并低速回安全零位。")
        self.last_error.set("")
        self.start_button.config(state="disabled")
        self.recovery_button.config(state="disabled")

    def consume_applied_offsets(self, verified):
        if not verified or self.stream is None:
            return None
        rows = tail_rows(self.stream)
        sample = next((r for r in reversed(rows) if r.get("kind") == "sample"), None)
        if not sample or "joint_offset_progress_deg" not in sample:
            return None
        preferences = ROOT / "config/uarm_cr5_continuous_ui.json"
        saved = json.loads(preferences.read_text(encoding="utf-8"))
        requested = [float(v) for v in saved.get("joint_offsets_deg", [0.0]*6)]
        progress = [float(v) for v in sample["joint_offset_progress_deg"]]
        if len(requested) != 6 or len(progress) != 6:
            return None
        remaining = [total-done for total, done in zip(requested, progress)]
        remaining = [0.0 if abs(value) < .05 else value for value in remaining]
        saved["joint_offsets_deg"] = remaining
        pending = preferences.with_suffix(".tmp")
        pending.write_text(json.dumps(saved, indent=2), encoding="utf-8")
        pending.replace(preferences)
        for variable, value in zip(self.offsets, remaining):
            variable.set(f"{value:g}")
        return remaining

    def finish(self, code, terminal):
        self.process = None
        if self.console:
            self.console.close()
            self.console = None
        cleanup = terminal.get("cleanup", {}) if terminal else {}
        verified = cleanup.get("cr5_final_disable_verified")
        remaining_offsets = self.consume_applied_offsets(verified)
        if code == 0 and verified:
            if cleanup.get("automatic_return"):
                self.status.set("已返回操作归零位，并确认 CR5 失能。")
            else:
                self.status.set("已立即停止并确认 CR5 失能；未执行自动回位。")
        else:
            self.status.set("控制程序已结束；请查看下方状态与日志。")
        if terminal:
            error = terminal.get("error") or terminal.get("escaped_error")
            if error:
                self.last_error.set(str(error))
            state = cleanup.get("cr5_final")
            if state:
                self.detail.set(f"结束状态：模式 {state['robot_mode']}，使能 {state['enable_status']}，报警 {state['error_status']}。")
                if remaining_offsets and any(abs(value) > .05 for value in remaining_offsets):
                    pending_text = ", ".join(
                        f"J{i+1} {value:+.1f}°" for i, value in enumerate(remaining_offsets)
                        if abs(value) > .05
                    )
                    self.detail.set(self.detail.get() + f" 待继续偏置：{pending_text}。")
        elif code:
            self.last_error.set("程序启动或预检失败；打开本轮日志中的 console.txt 查看原因。")
        uncertain_stop = not verified and (terminal is None or cleanup.get("enable_requested", True))
        self.uncertain_stop = uncertain_stop
        if uncertain_stop:
            self.last_error.set("未确认 CR5 失能，请使用实体急停并核对设备状态；本窗口禁止再次启动。")
        self.start_button.config(state="disabled" if uncertain_stop else "normal")
        self.recovery_button.config(state="normal")
        self.return_button.config(state="disabled")
        self.stop_button.config(state="disabled")
        self.stop_mode = None
        for widget in self.editable:
            widget.config(state="normal")

    def open_viewer(self):
        if os.name == "nt":
            handle = ctypes.windll.user32.FindWindowW(None, VIEWER_TITLE)
            if handle:
                ctypes.windll.user32.ShowWindow(handle, 9)
                ctypes.windll.user32.SetForegroundWindow(handle)
                return
        log = (ROOT / "logs/live_viewer_panel_console.txt").open("a", encoding="utf-8")
        try:
            subprocess.Popen([sys.executable, str(ROOT / "src/uarm_live_log_viewer.py")], cwd=ROOT,
                             stdout=log, stderr=subprocess.STDOUT,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        finally:
            log.close()

    def open_logs(self):
        os.startfile(str(self.heartbeat.parent if self.heartbeat else ROOT / "logs"))

    def close(self):
        if self.recovery_process is not None:
            self.last_error.set("碰撞恢复正在执行；如需立即停止，请使用实体急停。")
        elif self.process is None:
            self.root.destroy()
        else:
            self.closing = True
            self.emergency_stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ui-smoke-test", action="store_true", help="open and close UI without launching any controller")
    parser.add_argument("--autostart", action="store_true", help="start the current saved live settings after the panel opens")
    parser.add_argument("--confirm-autostart", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.autostart and args.confirm_autostart != CONFIRM:
        parser.error("--autostart requires the continuous-motion confirmation token")
    mutex = None
    if os.name == "nt":
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
        kernel.CreateMutexW.restype = ctypes.c_void_p
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        mutex = kernel.CreateMutexW(None, False, "Local\\UArmCR5ContinuousPanel")
        if not mutex:
            raise ctypes.WinError(ctypes.get_last_error())
        if ctypes.get_last_error() == 183:
            kernel.CloseHandle(mutex)
            handle = ctypes.windll.user32.FindWindowW(None, "U-Arm → CR5 真机连续联动")
            if handle:
                ctypes.windll.user32.ShowWindow(handle, 9)
                ctypes.windll.user32.SetForegroundWindow(handle)
            return
    try:
        root = tk.Tk()
        panel = LivePanel(root)
        if args.ui_smoke_test:
            root.after(750, root.destroy)
        elif args.autostart:
            root.after(750, panel.start)
        root.mainloop()
    finally:
        if mutex:
            kernel.CloseHandle(mutex)


if __name__ == "__main__":
    main()
