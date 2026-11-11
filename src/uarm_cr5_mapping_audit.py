"""File-only live view of U-Arm/CR5 joint mapping; opens no hardware links."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs"
CONTROL_CONFIG = ROOT / "config" / "uarm_cr5_continuous_ui.json"
INPUT_CONFIG = ROOT / "config" / "uarm_dobot_teleop.json"
MAX_AGE_S = 2.0
PAIR_LOG = LOG_DIR / "uarm_cr5_mapping_pairs.jsonl"


def sample_rows(path: Path) -> list[dict]:
    with path.open("rb") as stream:
        stream.seek(0, 2)
        end = stream.tell()
        stream.seek(max(0, end - 256_000))
        lines = stream.read().splitlines()
    rows = []
    for line in lines:
        try:
            candidate = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            continue
        if candidate.get("kind") == "sample":
            rows.append(candidate)
    return rows[-40:]


def latest_sample() -> tuple[Path, dict, float]:
    files = sorted(LOG_DIR.glob("uarm_cr5_dual_twin_*.jsonl"),
                   key=lambda path: path.stat().st_mtime, reverse=True)
    if not files:
        raise ValueError("没有找到双臂实时监视日志；请先打开只读双臂窗口。")
    path = files[0]
    rows = sample_rows(path)
    if not rows:
        raise ValueError("实时日志里还没有完整采样。")
    row = rows[-1]
    timestamp = datetime.fromisoformat(row["timestamp_utc"])
    age = (datetime.now(timezone.utc) - timestamp).total_seconds()
    return path, row, age


def relative_ticks(current: int, reference: int) -> int:
    return ((int(current) - int(reference) + 2048) % 4096) - 2048


def stable_for_capture(path: Path, latest: dict) -> bool:
    recent = []
    latest_time = datetime.fromisoformat(latest["timestamp_utc"])
    for row in sample_rows(path):
        age = (latest_time - datetime.fromisoformat(row["timestamp_utc"])).total_seconds()
        if 0.0 <= age <= 2.2:
            recent.append(row)
    if len(recent) < 2:
        return False
    span = (datetime.fromisoformat(recent[-1]["timestamp_utc"])
            - datetime.fromisoformat(recent[0]["timestamp_utc"])).total_seconds()
    if span < 0.5:
        return False
    for axis in range(6):
        values = [int(row["uarm_ticks"][axis]) for row in recent]
        relative = [relative_ticks(value, values[-1]) for value in values]
        if max(relative) - min(relative) > 4:
            return False
    for axis in range(6):
        values = [float(row["cr5_actual_joint_deg"][axis]) for row in recent]
        if not all(math.isfinite(value) for value in values) or max(values) - min(values) > 0.1:
            return False
    return True


def load_pair_points() -> dict[str, dict]:
    points: dict[str, dict] = {}
    if not PAIR_LOG.exists():
        return points
    with PAIR_LOG.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            label = row.get("point")
            if label in {"A", "B"}:
                points[label] = row
    return points


def estimate_pair(points: dict[str, dict], sources: list[int]) -> list[str]:
    if "A" not in points or "B" not in points:
        return ["记录点 A 和点 B 后，这里会按每轴角度变化计算建议方向与倍率。"]
    a, b = points["A"], points["B"]
    result = ["两点估算（输入 U-Arm 相对角变化 → CR5 实测关节变化；每轴独立判断）："]
    for i, source_id in enumerate(sources):
        source = int(source_id) - 1
        du_ticks = relative_ticks(b["uarm_ticks"][source], a["uarm_ticks"][source])
        du_deg = du_ticks * 360.0 / 4096.0
        dc_deg = float(b["cr5_actual_joint_deg"][i]) - float(a["cr5_actual_joint_deg"][i])
        if abs(du_deg) < 1.0:
            estimate = "U-Arm变化不足1°，无法可靠估倍率"
        elif abs(dc_deg) < 0.2:
            estimate = "CR5变化不足0.2°，方向/倍率不可判"
        else:
            sign = 1 if du_deg * dc_deg > 0 else -1
            scale = abs(dc_deg / du_deg)
            estimate = f"建议方向 {sign:+d}，倍率 {scale:.3f}"
            if not 0.1 <= scale <= 2.0:
                estimate += "（超出仿真控件0.1–2范围）"
        other_inputs = [
            abs(relative_ticks(b["uarm_ticks"][j], a["uarm_ticks"][j]) * 360.0 / 4096.0)
            for j in range(6) if j != source
        ]
        if other_inputs and max(other_inputs) > 2.0:
            estimate += "；点间其他主臂轴也变化>2°，需逐轴重测"
        other_outputs = [
            abs(float(b["cr5_actual_joint_deg"][j]) - float(a["cr5_actual_joint_deg"][j]))
            for j in range(6) if j != i
        ]
        if other_outputs and max(other_outputs) > 2.0:
            estimate += "；点间其他CR5轴也变化>2°，需逐轴重测"
        result.append(
            f"J{i+1} ← U-Arm ID{source_id}: ΔU={du_deg:+.2f}°，"
            f"ΔCR5={dc_deg:+.2f}°；{estimate}"
        )
    if not (a.get("cr5_feedback_fresh_verified") and b.get("cr5_feedback_fresh_verified")):
        result.append("注意：当前日志的 CR5 独立实时性标记未验证；估算仅供仿真预览，不作为真机控制参数。")
    return result


def mapping_preview(row: dict, base_ticks: list[int], base_deg: list[float]) -> tuple[list[dict], dict]:
    control = json.loads(CONTROL_CONFIG.read_text(encoding="utf-8"))
    input_config = json.loads(INPUT_CONFIG.read_text(encoding="utf-8"))
    reference = row.get("simulation_reference") or {}
    ticks = row.get("uarm_ticks")
    actual = row.get("cr5_actual_joint_deg")
    display = row.get("uarm_display_angles_deg")
    sim = row.get("cr5_preview_joint_deg")
    if not (base_ticks and base_deg and ticks and actual and display and sim):
        raise ValueError("当前日志缺少 U-Arm/CR5 配对参考字段。")
    signs = control["joint_signs"]
    sources = control["joint_sources"]
    scales = control["joint_scales"]
    offsets = control.get("joint_offsets_deg", [0.0] * 6)
    selected = set(control.get("joints", []))
    deadband = input_config.get("input_calibration", {}).get(
        "joint_input_deadband_ticks", [0] * 6)
    rows = []
    for i in range(6):
        source = int(sources[i]) - 1
        raw_ticks = relative_ticks(ticks[source], base_ticks[source])
        db = int(deadband[i])
        effective_ticks = math.copysign(max(0, abs(raw_ticks) - db), raw_ticks)
        leader_delta_deg = effective_ticks * 360.0 / 4096.0
        mapped_delta_deg = float(signs[i]) * float(scales[i]) * leader_delta_deg
        if i + 1 in selected:
            requested = float(base_deg[i]) + float(offsets[i]) + mapped_delta_deg
        else:
            requested = float(base_deg[i])
            mapped_delta_deg = 0.0
        rows.append({
            "joint": i + 1,
            "source": source + 1,
            "ticks": int(ticks[source]),
            "anchor_ticks": int(base_ticks[source]),
            "display_deg": float(display[source]),
            "leader_delta_deg": leader_delta_deg,
            "mapped_delta_deg": mapped_delta_deg,
            "requested_deg": requested,
            "actual_deg": float(actual[i]),
            "sim_deg": float(sim[i]),
            "selected": i + 1 in selected,
            "sign": int(signs[i]),
            "scale": float(scales[i]),
            "offset": float(offsets[i]),
            "discontinuity": abs(raw_ticks) > 512,
        })
    summary = {
        "control": control,
        "anchor_reason": reference.get("reason", "unknown"),
        "sim_signs": reference.get("signs", []),
        "sim_scales": reference.get("scales", []),
        "feedback_verified": bool(row.get("cr5_feedback_fresh_verified", False)),
    }
    return rows, summary


class MappingAudit:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("U-Arm → CR5 映射只读审计（当前实测参考）")
        self.root.geometry("1260x520")
        self.status = tk.StringVar(value="读取实时日志……")
        self.config_status = tk.StringVar(value="")
        self.anchor_path: Path | None = None
        self.anchor_ticks: list[int] | None = None
        self.anchor_deg: list[float] | None = None
        self.anchor_version: int | None = None
        self.anchor_label = ""
        self.points = load_pair_points()
        ttk.Label(self.root, text="U-Arm → CR5 关节映射预览（只读）",
                  font=("Microsoft YaHei", 16, "bold")).pack(anchor="w", padx=12, pady=(10, 2))
        ttk.Label(self.root, textvariable=self.status, wraplength=1220).pack(anchor="w", padx=12, pady=3)
        columns = ("joint", "source", "ticks", "anchor", "u_model", "u_delta",
                   "map_delta", "requested", "cr5_actual", "sim_target", "settings")
        self.table = ttk.Treeview(self.root, columns=columns, show="headings", height=7)
        headings = {
            "joint": "CR5轴", "source": "U-Arm来源ID", "ticks": "当前ticks",
            "anchor": "配对ticks", "u_model": "U-Arm模型角°",
            "u_delta": "相对变化°", "map_delta": "映射变化°",
            "requested": "实机公式目标°", "cr5_actual": "CR5实测°",
            "sim_target": "绿色仿真目标°", "settings": "当前映射设置",
        }
        widths = {"joint": 58, "source": 92, "ticks": 86, "anchor": 86,
                  "u_model": 108, "u_delta": 98, "map_delta": 98,
                  "requested": 110, "cr5_actual": 105, "sim_target": 110, "settings": 220}
        for col in columns:
            self.table.heading(col, text=headings[col])
            self.table.column(col, width=widths[col], anchor="center", stretch=False)
        self.table.pack(fill="x", padx=12, pady=6)
        capture_bar = ttk.Frame(self.root)
        capture_bar.pack(fill="x", padx=12, pady=(2, 4))
        ttk.Button(capture_bar, text="记录配对点 A（静止）",
                   command=lambda: self.capture_point("A")).pack(side="left", padx=(0, 6))
        ttk.Button(capture_bar, text="记录配对点 B（静止）",
                   command=lambda: self.capture_point("B")).pack(side="left", padx=6)
        self.point_status = tk.StringVar(value="尚未记录 A/B 配对点")
        ttk.Label(capture_bar, textvariable=self.point_status).pack(side="left", padx=10)
        self.estimate_status = tk.StringVar(value="")
        ttk.Label(self.root, textvariable=self.estimate_status, wraplength=1220,
                  justify="left").pack(anchor="w", padx=12, pady=(2, 5))
        ttk.Label(self.root, textvariable=self.config_status, wraplength=1220,
                  justify="left").pack(anchor="w", padx=12, pady=4)
        ttk.Label(
            self.root,
            text=("说明：‘实机公式目标’按当前来源/方向/倍率/死区/偏置计算，默认锚点取本窗口启动时的U-Arm ticks和CR5实测角；"
                  "仿真里的‘工作候选’姿态不会被当作真机起点；点击‘当前主臂配对实机姿态’后才采用新的实测配对；"
                  "只作离线预览，不包含真机限速/限位处理，也不向机器人发送指令。"
                  "U-Arm模型角是显示配准结果，不等同于控制输入角。"),
            foreground="#8b3a00", wraplength=1220, justify="left",
        ).pack(anchor="w", padx=12, pady=(3, 8))
        self.root.geometry("1260x650")
        self.root.after(250, self.refresh)

    def capture_point(self, label: str) -> None:
        try:
            path, row, age = latest_sample()
            if not -1.0 <= age <= 1.5:
                raise ValueError(f"最新采样过期 {age:.2f}s；不能记录。")
            if not stable_for_capture(path, row):
                raise ValueError("两臂至少连续静止0.8秒后再记录。")
            point = {
                "point": label,
                "timestamp_utc": row["timestamp_utc"],
                "log_file": path.name,
                "sample": row["sample"],
                "uarm_ticks": list(row["uarm_ticks"][:6]),
                "uarm_display_angles_deg": list(row["uarm_display_angles_deg"][:6]),
                "cr5_actual_joint_deg": list(row["cr5_actual_joint_deg"]),
                "cr5_feedback_fresh_verified": bool(row.get("cr5_feedback_fresh_verified", False)),
                "cr5_robot_mode": row.get("cr5_robot_mode"),
                "cr5_enable_status": row.get("cr5_enable_status"),
                "cr5_error_status": row.get("cr5_error_status"),
                "note": "paired measurement only; never applied to hardware",
            }
            with PAIR_LOG.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(point, ensure_ascii=False, allow_nan=False) + "\n")
            self.points[label] = point
            self.point_status.set(
                f"点{label} 已记录 | CR5实时性标记："
                f"{'已验证' if point['cr5_feedback_fresh_verified'] else '未验证'} | "
                f"模式{point['cr5_robot_mode']} 使能{point['cr5_enable_status']} 报警{point['cr5_error_status']}"
            )
            cfg = json.loads(CONTROL_CONFIG.read_text(encoding="utf-8"))
            self.estimate_status.set("\n".join(estimate_pair(self.points, cfg["joint_sources"])))
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            self.point_status.set(f"点{label} 未记录：{exc}")

    def refresh(self) -> None:
        try:
            path, row, age = latest_sample()
            reference = row.get("simulation_reference") or {}
            version = reference.get("version")
            reason = reference.get("reason")
            if self.anchor_ticks is None or self.anchor_path != path:
                # Start a separate mapping preview from the real feedback at
                # viewer startup. It must not inherit a virtual work-candidate
                # pose from the simulation controls.
                self.anchor_ticks = list(row["uarm_ticks"][:6])
                self.anchor_deg = list(row["cr5_actual_joint_deg"])
                self.anchor_path = path
                self.anchor_version = version
                self.anchor_label = "审计窗口启动时的 U-Arm/CR5 实测配对"
            elif reason in {"actual", "startup_actual"} and version != self.anchor_version:
                # The twin's explicit "pair actual" action is a valid new
                # measured anchor. Candidate and direction-only rebases are
                # simulation state and are intentionally ignored here.
                self.anchor_ticks = list(reference["leader_anchor_ticks"][:6])
                self.anchor_deg = list(reference["robot_anchor_deg"])
                self.anchor_version = version
                self.anchor_label = f"按实测姿态重新配对（仿真参考版本 {version}）"
            rows, summary = mapping_preview(row, self.anchor_ticks, self.anchor_deg)
            children = self.table.get_children()
            if children:
                self.table.delete(*children)
            for item in rows:
                settings = ("跟随" if item["selected"] else "保持基准")
                settings += f" | s={item['sign']:+d} k={item['scale']:g} b={item['offset']:g}°"
                if item["discontinuity"]:
                    settings += " | 增量超45°"
                self.table.insert("", "end", values=(
                    f"J{item['joint']}", f"ID{item['source']}", item["ticks"],
                    item["anchor_ticks"], f"{item['display_deg']:+.2f}",
                    f"{item['leader_delta_deg']:+.2f}", f"{item['mapped_delta_deg']:+.2f}",
                    f"{item['requested_deg']:+.2f}", f"{item['actual_deg']:+.2f}",
                    f"{item['sim_deg']:+.2f}", settings,
                ))
            selected = ",".join(str(v) for v in summary["control"].get("joints", [])) or "无"
            age_text = f"新鲜 {age:.2f}s" if -1 <= age <= MAX_AGE_S else f"过期 {age:.1f}s"
            status = (f"日志 {path.name} | {age_text} | CR5 模式 {row.get('cr5_robot_mode')} / "
                      f"使能 {row.get('cr5_enable_status')} / 报警 {row.get('cr5_error_status')} | "
                      f"CR5反馈独立实时性标记={'已验证' if summary['feedback_verified'] else '未验证'} | "
                      f"仿真参考={summary['anchor_reason']} | 公式预览锚点：{self.anchor_label}")
            if age > MAX_AGE_S or age < -1:
                status += " | 当前日志不可作为实时标定数据"
            dm_error = row.get("dm_poll_error")
            if dm_error:
                status += f" | DM反馈不可用：{dm_error}"
            sim_signs = summary.get("sim_signs", [])
            sim_text = ",".join(f"{int(v):+d}" for v in sim_signs) if sim_signs else "未知"
            sim_scales = summary.get("sim_scales", [])
            sim_scale_text = ",".join(f"{float(v):g}" for v in sim_scales) if sim_scales else "未知"
            cfg = summary["control"]
            ctrl_signs = ",".join(f"{int(v):+d}" for v in cfg["joint_signs"])
            scales = ",".join(f"{float(v):g}" for v in cfg["joint_scales"])
            sources = ",".join(str(v) for v in cfg["joint_sources"])
            self.config_status.set(
                f"控制映射配置：启用CR5轴 [{selected}]；来源ID [{sources}]；"
                f"方向 [{ctrl_signs}]；倍率 [{scales}]。仿真方向 [{sim_text}]；"
                f"仿真倍率 [{sim_scale_text}]。"
                "控制配置与仿真配置分别存储；差异不会自动同步。"
            )
            self.estimate_status.set("\n".join(
                estimate_pair(self.points, cfg["joint_sources"])))
            if not self.points:
                self.point_status.set("尚未记录 A/B 配对点")
            self.status.set(status)
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            self.status.set(f"数据不可用：{exc}")
            self.config_status.set("")
        self.root.after(250, self.refresh)


if __name__ == "__main__":
    MappingAudit().root.mainloop()
