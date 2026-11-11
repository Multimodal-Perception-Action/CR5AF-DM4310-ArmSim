"""Offline comparison of CR5 joint zero and the recorded work-pose candidate.

Only local models/configuration are read. No controller, servo, or motor access.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from cr5_mujoco_twin import load_model, apply_degrees, link6_pose
from uarm_cr5_dual_twin import build_scene, set_joint_rad

ROOT = Path(__file__).resolve().parents[1]


def inspect_pose(model, data, addresses, q):
    if len(q) != 6 or not all(math.isfinite(x) for x in q):
        raise ValueError('Expected six finite joint angles')
    margins = []
    for index, angle in enumerate(q, 1):
        joint = model.joint(f'joint{index}').id
        lo, hi = np.degrees(model.jnt_range[joint])
        if not lo <= angle <= hi:
            raise ValueError(f'J{index} outside model limits')
        margins.append(float(min(angle-lo, hi-angle)))
    apply_degrees(model, data, addresses, q)
    contacts = []
    adjacent_mount_contacts = []
    for contact in data.contact:
        one, two = int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])
        row = {'body1':model.body(one).name, 'body2':model.body(two).name,
               'distance_m':float(contact.dist)}
        if int(model.body_parentid[one]) == two or int(model.body_parentid[two]) == one:
            adjacent_mount_contacts.append(row)
        else:
            contacts.append(row)
    return {'q_deg_recorded':q, 'inside_official_model_limits':True,
            'limit_margin_deg':margins, 'model_link6_pose':link6_pose(model,data),
            'reported_nonadjacent_body_contacts':contacts,
            'adjacent_mount_contacts':adjacent_mount_contacts}


def make_preview():
    config = json.loads((ROOT/'config/uarm_cr5_full_calibration.json').read_text(encoding='utf-8'))
    robot = config['robot']
    poses = [robot['controller_origin']['joint_deg'],
             robot['safe_home_candidate']['joint_deg_controller_representation']]
    collision_model, collision_data, addresses = load_model()
    checks = [inspect_pose(collision_model,collision_data,addresses,q) for q in poses]
    model, data = build_scene()
    width, height = 570, 540
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = [.42, -.10, .50]
    camera.distance = 2.05
    camera.azimuth = 35
    camera.elevation = -20
    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    option.geomgroup[:] = [True,False,True,False,False,False]
    sheet = Image.new('RGB',(width*2, height+140),'#131820')
    draw = ImageDraw.Draw(sheet)
    font_file = Path('C:/Windows/Fonts/msyh.ttc')
    font = ImageFont.truetype(str(font_file),19) if font_file.exists() else ImageFont.load_default()
    small = ImageFont.truetype(str(font_file),15) if font_file.exists() else font
    with mujoco.Renderer(model,height=height,width=width) as renderer:
        for col,q in enumerate(poses):
            for i,value in enumerate(q,1):
                set_joint_rad(model,data,f'a_joint{i}',math.radians(value))
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera,scene_option=option)
            sheet.paste(Image.fromarray(renderer.render().copy()),(width*col,65))
    draw.text((20,14),'CR5 关节零位参考',font=font,fill='#86b7ff')
    draw.text((width+20,14),'已记录的工作姿态候选（尚未确认）',font=font,fill='#ffbf75')
    draw.text((20,43),'J1–J6 = 0°；角度基准，不是活动范围中点',font=small,fill='white')
    draw.text((width+20,43),'用于与 U-Arm 舒适中位建立对应',font=small,fill='white')
    draw.text((20,height+78),'两图均为离线模型：没有移动真实机械臂。',font=font,fill='white')
    draw.text((20,height+108),'夹爪为示意；环境障碍、工具安装及到达路径尚未验证。',font=small,fill='#d5dce7')
    output = ROOT/'logs/cr5_work_home_comparison.png'
    sheet.save(output)
    report = {
        'created_at_utc':datetime.now(timezone.utc).isoformat(),
        'hardware_access':False,'poses':dict(zip(['joint_zero','recorded_candidate'],checks)),
        'image':str(output),'candidate_confirmed':False,'path_checked':False,
        'limitations':[
            'Local generic CR5 URDF; not independently calibrated to this CR5AF and tool.',
            'Collision query used original body mesh proxies; rendering model collision is disabled.',
            'Gripper, cables, table and surrounding obstacles are not included in the contact query.',
            'No-contact result is a coarse static-model observation, not hardware clearance approval.',
            'J1 312.2704 and -47.7296 render identically but must not be exchanged in a motion command without path checks.',
            'No transition trajectory or combined workspace has been validated.'
        ]}
    (ROOT/'logs/cr5_work_home_preview.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'image':str(output),'hardware_access':False,
                      'model_joint_limits_ok':True,'candidate_confirmed':False}))
    return output


def show(path):
    import ctypes
    import tkinter as tk
    from PIL import ImageTk
    try:
        ctypes.windll.user32.SetThreadDpiAwarenessContext(ctypes.c_void_p(-4))
    except (AttributeError,OSError):
        pass
    root = tk.Tk()
    root.title('CR5 工作中位对比 — 纯离线预览')
    root.tk.call('tk','scaling',1.0)
    root.geometry('+45+45')
    root.resizable(False,False)
    with Image.open(path) as original:
        photo = ImageTk.PhotoImage(original.copy())
    label = tk.Label(root,image=photo,borderwidth=0)
    label.pack()
    root.mainloop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--show-existing',action='store_true')
    args = parser.parse_args()
    path = ROOT/'logs/cr5_work_home_comparison.png' if args.show_existing else make_preview()
    if args.gui:
        show(path)
