"""Read-only calibration panel. Consumes the existing twin log, never hardware."""
from __future__ import annotations

import json
import math
import argparse
from datetime import datetime, timezone
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "logs"
OUT = LOGS / f"calibration_panel_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl"
MAX_AGE = 2.0


def capture_leader_position(axis: int, role: str) -> dict:
    """Capture one stable leader position from the existing read-only twin log."""
    if axis not in range(1, 8):
        raise ValueError("axis must be within 1..7")
    if role not in {"center", "side_a", "side_b"}:
        raise ValueError("role must be center, side_a, or side_b")
    path, rows, _age = samples()
    row = rows[-1]
    recent = [
        item for item in rows
        if (datetime.fromisoformat(row["timestamp_utc"])
            - datetime.fromisoformat(item["timestamp_utc"])).total_seconds() <= 0.8
    ]
    if len(recent) < 3 or (
        datetime.fromisoformat(recent[-1]["timestamp_utc"])
        - datetime.fromisoformat(recent[0]["timestamp_utc"])
    ).total_seconds() < 0.3:
        raise ValueError("有效采样不足，请保持静止后重试。")
    values = [int(item["uarm_ticks"][axis - 1]) for item in recent]
    wrapped = [((value - values[-1] + 2048) % 4096) - 2048 for value in values]
    if max(wrapped) - min(wrapped) > 4:
        raise ValueError("主臂仍在移动，请保持静止后重试。")
    event = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "source_log": path.name,
        "source_sample": row["sample"],
        "axis": axis,
        "target": "leader",
        "role": role,
        "value": values[-1],
        "unit": "ticks",
        "peak_to_peak_ticks": max(wrapped) - min(wrapped),
        "robot_mode": row["cr5_robot_mode"],
        "robot_enabled": row["cr5_enable_status"],
        "robot_error": row["cr5_error_status"],
        "note": "measurement only; not applied to control",
    }
    with OUT.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
    return event


def samples():
    files = sorted(LOGS.glob("uarm_cr5_dual_twin_*.jsonl"), reverse=True)
    if not files:
        raise ValueError("找不到实时窗口的数据；请启动只读双机窗口。")
    with files[0].open("rb") as f:
        f.seek(0, 2)
        end = f.tell()
        f.seek(max(0, end - 100000))
        lines = f.read().splitlines()
    rows = []
    for line in lines:
        try:
            row = json.loads(line)
            if row.get("kind") == "sample":
                rows.append(row)
        except (ValueError, UnicodeDecodeError):
            continue
    if not rows:
        raise ValueError("实时窗口暂无完整数据。")
    latest = rows[-1]
    age = (datetime.now(timezone.utc) - datetime.fromisoformat(latest["timestamp_utc"])).total_seconds()
    if age < -1 or age > MAX_AGE:
        raise ValueError(f"实时数据已过期 {age:.1f} 秒，不能记录。")
    return files[0], rows[-12:], age


class Panel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("U-Arm / CR5AF 逐轴标定（只读记录）")
        self.root.geometry("1100x720")
        self.axis = tk.IntVar(value=2)
        self.status = tk.StringVar(value="正在读取实时数据……")
        self.detail = tk.StringVar(value="")
        self.saved = {}
        self.j1 = json.loads((LOGS / "j1_calibration_20261110.json").read_text(encoding="utf-8"))
        self.grip = json.loads((ROOT / "config" / "uarm_gripper_calibration.json").read_text(encoding="utf-8"))
        title = ttk.Label(self.root, text="U-Arm / CR5AF 逐轴标定", font=("Microsoft YaHei", 18, "bold"))
        title.pack(pady=6)
        ttk.Label(self.root, text="只读记录：选轴 → 手动摆好 → 点击对应记录。不会移动、使能或修改机器人。", font=("Microsoft YaHei", 10)).pack()
        ttk.Label(self.root, textvariable=self.status, foreground="#075e9b", font=("Microsoft YaHei", 11, "bold")).pack(pady=5)
        bar = ttk.Frame(self.root); bar.pack(pady=4)
        for i in range(1, 8):
            ttk.Radiobutton(bar, text=f"J{i}" if i < 7 else "夹爪 ID7", variable=self.axis, value=i, command=self.refresh_saved).pack(side="left", padx=10)
        self.tree = ttk.Treeview(self.root, columns=("id", "ticks", "deg", "robot"), show="headings", height=7)
        for col, label, width in [("id", "通道", 120), ("ticks", "主臂编码值", 200), ("deg", "单圈角度（度）", 220), ("robot", "CR5 关节（度）", 220)]:
            self.tree.heading(col, text=label); self.tree.column(col, width=width, anchor="center")
        for i in range(1, 8): self.tree.insert("", "end", iid=str(i), values=(f"J{i}" if i < 7 else "夹爪 ID7", "--", "--", "--"))
        self.tree.pack(fill="x", padx=20, pady=6)
        buttons = ttk.LabelFrame(self.root, text="记录选中通道的位置（每次独立保存）")
        buttons.pack(fill="x", padx=20, pady=5)
        for label, target, role in [("记录主臂中位", "leader", "center"), ("记录主臂左 / 上 / 张开", "leader", "side_a"), ("记录主臂右 / 下 / 闭合", "leader", "side_b"), ("记录 CR5 当前工作位", "robot", "center"), ("记录 CR5 一侧边界", "robot", "side_a"), ("记录 CR5 另一侧边界", "robot", "side_b")]:
            ttk.Button(buttons, text=label, command=lambda t=target, r=role: self.capture(t, r)).pack(side="left", expand=True, padx=3, pady=10)
        ttk.Label(self.root, textvariable=self.detail, justify="left", font=("Microsoft YaHei", 10)).pack(anchor="w", padx=24, pady=8)
        self.note = ttk.Label(self.root, text="夹爪电机反馈尚无独立时间戳，暂不支持界面记录电机端位置。此前数据仅供参考，不自动覆盖控制参数。", wraplength=1020)
        self.note.pack(anchor="w", padx=24)
        self.refresh_saved()
        self.root.after(100, self.update)

    def refresh_saved(self):
        i = self.axis.get()
        text = f"当前选择：{'夹爪 ID7' if i == 7 else f'J{i}'}。本次界面记录：{self.saved.get(i, {})}"
        if i == 1:
            text += f"\n此前 J1：中位 {self.j1['center_ticks']}，左 {self.j1['endpoint_a_ticks']}，右 {self.j1['endpoint_b_ticks']} ticks。"
        if i == 7:
            e = self.grip['endpoints']
            text += f"\n此前夹爪：张开 {e['maximum_open']['ticks']}，闭合 {e['fully_closed']['ticks']} ticks。"
        self.detail.set(text)

    def update(self):
        try:
            path, rows, age = samples(); row = rows[-1]
            for i in range(7):
                tick = int(row["uarm_ticks"][i]); robot = f"{row['cr5_actual_joint_deg'][i]:.2f}°" if i < 6 else (f"DM {row['dm_feedback']['position_rad']:.3f} rad" if row.get('dm_feedback') and not row.get('dm_poll_error') else "DM 无有效反馈")
                self.tree.item(str(i+1), values=(f"J{i+1}" if i < 6 else "夹爪 ID7", tick, f"{tick*360/4096:.2f}°", robot))
            self.status.set(f"数据正常 | {age:.1f} 秒前 | CR5 模式 {row['cr5_robot_mode']} 使能 {row['cr5_enable_status']} 报警 {row['cr5_error_status']} | 反馈重连 {row['feedback_reconnects']}")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status.set(f"数据不可用：{exc}")
        self.root.after(250, self.update)

    def capture(self, target, role):
        try:
            path, rows, age = samples(); row = rows[-1]; i = self.axis.get()
            recent = [r for r in rows if (datetime.fromisoformat(row['timestamp_utc']) - datetime.fromisoformat(r['timestamp_utc'])).total_seconds() <= 0.8]
            if len(recent) < 3 or (datetime.fromisoformat(recent[-1]['timestamp_utc']) - datetime.fromisoformat(recent[0]['timestamp_utc'])).total_seconds() < 0.3:
                raise ValueError("有效采样不足，请保持静止后重试。")
            if target == "leader":
                vals = [int(r['uarm_ticks'][i-1]) for r in recent]
                delta = [((v-vals[-1]+2048)%4096)-2048 for v in vals]
                if max(delta)-min(delta) > 4: raise ValueError("主臂仍在移动，请保持静止后重试。")
                value = vals[-1]; unit = "ticks"
            elif i < 7:
                if not row.get('cr5_feedback_fresh_verified', False):
                    raise ValueError("CR5 原始反馈实时性尚未验证；请勿用此数据记录工作位或边界。")
                vals = [float(r['cr5_actual_joint_deg'][i-1]) for r in recent]
                if not all(math.isfinite(v) for v in vals) or max(vals)-min(vals) > 0.1: raise ValueError("CR5 数据无效或仍在移动。")
                value = vals[-1]; unit = "deg"
            else:
                raise ValueError("DM 反馈缺少独立时间戳；为避免记录过期位置，暂不支持记录电机端。")
            event = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "source_log": path.name, "source_sample": row['sample'], "axis": i, "target": target, "role": role, "value": value, "unit": unit, "robot_mode": row['cr5_robot_mode'], "robot_enabled": row['cr5_enable_status'], "robot_error": row['cr5_error_status'], "note": "measurement only; not applied to control"}
            with OUT.open("a", encoding="utf-8") as f: f.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+"\n")
            self.saved.setdefault(i, {})[f"{target}_{role}"] = round(value, 3)
            self.refresh_saved()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            messagebox.showwarning("无法记录", str(exc))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-leader", type=int, metavar="AXIS")
    parser.add_argument("--role", choices=("center", "side_a", "side_b"), default="center")
    args = parser.parse_args()
    if args.capture_leader is None:
        Panel().root.mainloop()
    else:
        print(json.dumps(capture_leader_position(args.capture_leader, args.role), ensure_ascii=False))
