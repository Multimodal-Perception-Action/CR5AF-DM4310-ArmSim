"""Local, display-only U-Arm sign/offset registration for a Tk viewer.

This module has no device, serial or network API.  Its JSON is deliberately
separate from the control calibration.  Importing/constructing the panel never
overwrites an existing file; only the user's Save button writes settings.

Usage::

    panel = VisualRegistrationPanel(parent)
    panel.pack(fill="x")
    display_deg = panel.map_angles(seven_encoder_relative_degrees)

The input reference pose must be the same one used when these display offsets
were chosen.  The model applies ``display = sign * input + offset`` without
changing encoder centres or any real hardware setting.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import ttk
from typing import Sequence


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "uarm_visual_registration.json"
)
AXIS_NAMES = tuple([f"J{i}" for i in range(1, 7)] + ["扳机 ID7"])
OFFSET_LIMIT_DEG = 720.0


def _seven_finite(values: Sequence[float], name: str) -> list[float]:
    if len(values) != 7:
        raise ValueError(f"{name} 必须包含 7 个值")
    out = [float(value) for value in values]
    if not all(math.isfinite(value) for value in out):
        raise ValueError(f"{name} 包含非有限值")
    return out


@dataclass
class DisplayRegistration:
    """Pure mapping state; usable without starting Tk or accessing a device."""

    signs: list[float] = field(default_factory=lambda: [1.0] * 7)
    offsets_deg: list[float] = field(default_factory=lambda: [0.0] * 7)
    confirmed_by_operator: list[bool] = field(default_factory=lambda: [False] * 7)

    def validate(self) -> None:
        signs = _seven_finite(self.signs, "方向")
        offsets = _seven_finite(self.offsets_deg, "显示偏置")
        if any(value not in (-1.0, 1.0) for value in signs):
            raise ValueError("每轴方向只能为 +1 或 -1")
        if any(abs(value) > OFFSET_LIMIT_DEG for value in offsets):
            raise ValueError(f"显示偏置必须在 ±{OFFSET_LIMIT_DEG:g}° 内")
        if len(self.confirmed_by_operator) != 7 or any(
            type(value) is not bool for value in self.confirmed_by_operator
        ):
            raise ValueError("人工核对标记必须包含 7 个布尔值")

    def map_angles(self, relative_deg: Sequence[float]) -> list[float]:
        self.validate()
        angles = _seven_finite(relative_deg, "输入角度")
        result = [
            self.signs[i] * angles[i] + self.offsets_deg[i] for i in range(7)
        ]
        if not all(math.isfinite(value) for value in result):
            raise ValueError("映射后的显示角度超出有限数值范围")
        return result

    def as_dict(self) -> dict:
        self.validate()
        return {
            "schema_version": 1,
            "purpose": "display_only",
            "signs": list(self.signs),
            "offsets_deg": list(self.offsets_deg),
            "confirmed_by_operator": list(self.confirmed_by_operator),
            "input_reference": "caller-provided encoder relative degrees; unchanged by this file",
            "note": "Manual visual settings only; not robot control or collision calibration.",
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
        }

    @classmethod
    def from_dict(cls, value: dict) -> "DisplayRegistration":
        if not isinstance(value, dict) or value.get("schema_version") != 1:
            raise ValueError("显示设置版本不支持")
        if value.get("purpose") != "display_only":
            raise ValueError("该文件不是独立显示设置")
        obj = cls(
            signs=_seven_finite(value["signs"], "方向"),
            offsets_deg=_seven_finite(value["offsets_deg"], "显示偏置"),
            confirmed_by_operator=list(value["confirmed_by_operator"]),
        )
        obj.validate()
        return obj


class VisualRegistrationPanel(ttk.Frame):
    """Embed in a viewer; controls affect only the seven returned display angles.

    Malformed existing JSON is kept untouched and Save is blocked until the
    underlying file is repaired and the panel reopened. Invalid intermediate Spinbox
    text retains the last valid mapping, rather than crashing the viewer loop.
    """

    def __init__(
        self,
        parent,
        *,
        config_path: Path = DEFAULT_CONFIG_PATH,
        input_reference: dict | None = None,
    ):
        super().__init__(parent)
        if input_reference is not None and not isinstance(input_reference, dict):
            raise ValueError("输入参考必须是字典或 None")
        # Freeze a JSON-safe copy so a caller changing its list of centres
        # cannot silently alter the identity of this registration session.
        self.input_reference = json.loads(
            json.dumps(input_reference, ensure_ascii=False, allow_nan=False)
        )
        self.config_path = Path(config_path)
        if self.config_path.resolve() != DEFAULT_CONFIG_PATH.resolve():
            raise ValueError("显示设置只能保存到独立 uarm_visual_registration.json")
        self.mapping = DisplayRegistration()
        self._load_error = ""
        self._dirty = False
        self._selected = 0
        self._loaded = False
        self._reference_notice = ""
        if self.config_path.exists():
            try:
                saved = json.loads(self.config_path.read_text(encoding="utf-8"))
                self.mapping = DisplayRegistration.from_dict(saved)
                self._loaded = True
                if (
                    "reference_signature" not in saved
                    or saved["reference_signature"] != self.input_reference
                ):
                    self.mapping.confirmed_by_operator = [False] * 7
                    self._dirty = True
                    self._reference_notice = "输入参考已改变或旧设置未记录参考，需逐轴复核"
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self._load_error = str(exc)

        self.axis_var = tk.StringVar(value=AXIS_NAMES[0])
        self.offset_var = tk.StringVar(value="0")
        self.reverse_var = tk.BooleanVar(value=False)
        self.verified_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar()
        ttk.Label(self, text="模型显示校正（仅画面）").grid(row=0, column=0, padx=4, sticky="w")
        self.selector = ttk.Combobox(
            self, textvariable=self.axis_var, values=AXIS_NAMES,
            state="readonly", width=10,
        )
        self.selector.grid(row=0, column=1, padx=4)
        self.selector.bind("<<ComboboxSelected>>", self._select_axis)
        ttk.Checkbutton(
            self, text="反向", variable=self.reverse_var, command=self._direction_changed
        ).grid(row=0, column=2, padx=4)
        ttk.Label(self, text="偏置 °").grid(row=0, column=3, padx=3)
        self.spin = ttk.Spinbox(
            self, textvariable=self.offset_var, from_=-OFFSET_LIMIT_DEG,
            to=OFFSET_LIMIT_DEG, increment=1, width=9, command=self._offset_changed,
        )
        self.spin.grid(row=0, column=4, padx=3)
        self.spin.bind("<KeyRelease>", self._offset_changed)
        self.spin.bind("<FocusOut>", self._offset_changed)
        for col, step in enumerate((-5, -1, 1, 5), start=5):
            ttk.Button(self, text=f"{step:+d}°", width=4,
                       command=lambda amount=step: self._nudge(amount)).grid(row=0, column=col, padx=2)
        ttk.Button(self, text="保存显示设置", command=self.save).grid(row=0, column=9, padx=5)
        ttk.Checkbutton(
            self, text="我已核对本轴方向和姿态", variable=self.verified_var,
            command=self._confirm_changed,
        ).grid(row=1, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Label(self, textvariable=self.status_var, wraplength=620).grid(
            row=1, column=4, columnspan=6, sticky="w", padx=4,
        )
        self._show_axis()

    def map_angles(self, relative_deg: Sequence[float]) -> list[float]:
        return self.mapping.map_angles(relative_deg)

    def _show_axis(self):
        i = self._selected
        self.offset_var.set(f"{self.mapping.offsets_deg[i]:g}")
        self.reverse_var.set(self.mapping.signs[i] < 0)
        self.verified_var.set(self.mapping.confirmed_by_operator[i])
        self._status()

    def _status(self, extra=""):
        count = sum(self.mapping.confirmed_by_operator)
        label = "未标定" if count < 7 else "7 轴由操作者勾选核对；未验证控制映射"
        saved = "有未保存修改" if self._dirty else ("已载入已保存设置" if self._loaded else "默认值，未保存")
        if self._load_error:
            saved = f"原配置读取失败，未覆盖：{self._load_error}"
        if self._reference_notice and count < 7:
            saved += f" | {self._reference_notice}"
        self.status_var.set(f"{label} | 人工核对 {count}/7 | {saved}" + (f" | {extra}" if extra else ""))

    def _changed(self):
        self.mapping.confirmed_by_operator[self._selected] = False
        self.verified_var.set(False)
        self._dirty = True
        self._status()

    def _read_offset(self):
        value = float(self.offset_var.get())
        if not math.isfinite(value) or abs(value) > OFFSET_LIMIT_DEG:
            raise ValueError("显示偏置范围为 ±720°")
        return value

    def _offset_changed(self, event=None):
        try:
            value = self._read_offset()
        except ValueError:
            self._status("输入未完成或超范围，画面保留上次有效值")
            return False
        if value != self.mapping.offsets_deg[self._selected]:
            self.mapping.offsets_deg[self._selected] = value
            self._changed()
        return True

    def _direction_changed(self):
        self.mapping.signs[self._selected] = -1.0 if self.reverse_var.get() else 1.0
        self._changed()

    def _confirm_changed(self):
        if not self._offset_changed():
            self.verified_var.set(False)
            return
        self.mapping.confirmed_by_operator[self._selected] = bool(self.verified_var.get())
        self._dirty = True
        self._status()

    def _select_axis(self, event=None):
        if not self._offset_changed():
            self.axis_var.set(AXIS_NAMES[self._selected])
            return
        self._selected = AXIS_NAMES.index(self.axis_var.get())
        self._show_axis()

    def _nudge(self, amount):
        if not self._offset_changed():
            return
        value = self.mapping.offsets_deg[self._selected] + amount
        if abs(value) <= OFFSET_LIMIT_DEG:
            self.offset_var.set(f"{value:g}")
            self._offset_changed()

    def save(self):
        if not self._offset_changed():
            return False
        if self._load_error:
            self._status("保存已阻止，请先检查原配置文件；没有覆盖它")
            return False
        temp_path = None
        try:
            record = self.mapping.as_dict()
            record["reference_signature"] = self.input_reference
            payload = json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False)
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self.config_path.parent,
                prefix=".uarm_visual_registration_", suffix=".tmp", delete=False,
            ) as stream:
                temp_path = Path(stream.name)
                stream.write(payload + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.config_path)
            self._loaded = True
            self._dirty = False
            self._status("已保存，仅影响仿真显示")
            return True
        except (OSError, ValueError) as exc:
            self._status(f"保存失败：{exc}")
            return False
        finally:
            if temp_path is not None and temp_path.exists():
                try:
                    temp_path.unlink()
                except OSError:
                    pass
