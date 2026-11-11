"""Render guarded teleoperation telemetry; local files only, no device access."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import mujoco

from uarm_cr5_dual_twin import build_scene, set_joint_rad, set_tool_open_fraction, TwinCanvas
from uarm_visual_registration import DisplayRegistration
from uarm_dobot_preview import shortest_tick_step

ROOT = Path(__file__).resolve().parents[1]


def read_stream():
    files = sorted((ROOT/'logs').glob('uarm_cr5_control_stream_*.jsonl'), reverse=True)
    if not files:
        return None, [], None
    path = files[0]
    with path.open('rb') as stream:
        stream.seek(0, 2)
        stream.seek(max(0, stream.tell()-150000))
        content = stream.read().splitlines()
    rows = []
    for line in content:
        try:
            rows.append(json.loads(line))
        except (ValueError, UnicodeDecodeError):
            pass
    return path, rows, path.stat().st_mtime


def main():
    calibration = json.loads((ROOT/'config/uarm_cr5_full_calibration.json').read_text(encoding='utf-8'))
    display_config = json.loads((ROOT/'config/uarm_visual_registration.json').read_text(encoding='utf-8'))
    mapping = DisplayRegistration.from_dict(display_config)
    center = calibration['leader']['center_candidate_ticks']
    close_tick = json.loads((ROOT/'config/uarm_gripper_calibration.json').read_text(encoding='utf-8'))['endpoints']['fully_closed']['ticks']
    model, data = build_scene()
    gui = None
    current_file = None
    previous = None
    accumulated = None
    last_key = None
    last_sample = None
    try:
        gui = TwinCanvas(model, window_title='U-Arm / CR5 实时联动监视（文件读取）',
                         input_reference={'model':'uarm_output_axes_v1', 'center_ticks':center+[close_tick]})
        gui.registration.pack_forget()
        while gui.is_running():
            path, rows, _mtime = read_stream()
            if path != current_file:
                current_file = path
                previous = accumulated = last_sample = last_key = None
            samples = [r for r in rows if r.get('kind') == 'sample']
            terminal = next((r for r in reversed(rows) if r.get('kind') == 'terminal'), None)
            if samples:
                row = samples[-1]
                if terminal and terminal.get('cleanup',{}).get('cr5_final'):
                    row = dict(row, cr5=terminal['cleanup']['cr5_final'])
                key = (str(path), row['sample'])
                if terminal:
                    key = (*key, 'terminal')
                if key != last_key:
                    ticks = row['servo_ticks']
                    if previous is None:
                        accumulated = [shortest_tick_step(center[i],ticks[i]) for i in range(6)]
                    else:
                        for i in range(6):
                            accumulated[i] += shortest_tick_step(previous[i],ticks[i])
                    previous = ticks[:]
                    angles = mapping.map_angles([v*360/4096 for v in accumulated]
                                                 +[shortest_tick_step(close_tick,ticks[6])*360/4096])
                    for i,value in enumerate(angles):
                        set_joint_rad(model,data,f'u_joint{i+1}' if i<6 else 'u_trigger_joint',math.radians(value))
                    # This view always uses measured CR5 angles, never the command target.
                    for i,value in enumerate(row['cr5']['q_actual_deg'],1):
                        set_joint_rad(model,data,f'a_joint{i}',math.radians(value))
                    # Gateway OPEN/CLOSE command state is illustrated, not mislabelled as feedback.
                    if (row['gripper_desired'] in ('open','close')
                            and row.get('gripper_command_issued', True)):
                        set_tool_open_fraction(model,data,'a_',1.0 if row['gripper_desired']=='open' else 0.0)
                    mujoco.mj_forward(model,data)
                    last_key, last_sample = key, row
            if last_sample is None:
                title = '等待联动程序数据；本窗口不连接设备'
                status = '左：U-Arm | 右：CR5 实际反馈 | 夹爪画面显示开闭指令'
            else:
                row = last_sample
                age = (datetime.now(timezone.utc)-datetime.fromisoformat(row['timestamp_utc'])).total_seconds()
                state = '已结束，画面冻结' if terminal else ('数据已过期，画面冻结' if age>1 else '数据更新中')
                mode = '实机联动' if row.get('live_output') else '只读预演'
                title = f'{mode} | 左：U-Arm 实时输入 | 右：CR5 实测关节角 | {state}'
                scope = f"仅 J{row['selected_joint']}" if row.get('selected_joint') else '六轴'
                if row.get('selected_joints'):
                    scope = ','.join(f'J{j}' for j in row['selected_joints'])
                range_label = ('持续跟随 / 保留关节限位' if row.get('continuous')
                               else f"本轮 {scope} ±{row.get('arm_max_relative_deg',5)}°")
                gripper_label = ({'open':'张开','close':'闭合'}.get(row['gripper_desired'],'本轮不控制'))
                if row.get('continuous') and not row.get('gripper_command_issued', True):
                    gripper_label = '本轮尚未发送开闭指令'
                status = (f"实机：模式 {row['cr5']['robot_mode']} / 使能 {row['cr5']['enable_status']} / 报警 {row['cr5']['error_status']}"
                          f" | 数据年龄 {age:.1f} 秒\n"
                          f"夹爪：{gripper_label}（示意，非实测开度）"
                          f" | {range_label} / 最高 3°每秒\n"
                          '本窗口只读取日志；关闭监视窗口不会停止控制程序，现场使用实体急停')
            gui.update(data,title,status)
            time.sleep(.04)
    finally:
        if gui is not None:
            gui.close()


if __name__ == '__main__':
    main()
