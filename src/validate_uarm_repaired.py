"""Offline geometry acceptance and renders; no serial/network/device access."""
from pathlib import Path
import json
import math
import numpy as np
import mujoco
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'simulation/generated/uarm_repaired'


def main():
    scene = mujoco.MjSpec()
    scene.compiler.discardvisual = False
    scene.worldbody.add_light(pos=[0, -1, 2], dir=[0, .3, -1])
    scene.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[1, 1, .02], rgba=[.16, .17, .19, 1], contype=0, conaffinity=0)
    arm = mujoco.MjSpec.from_file(str(ASSETS / 'uarm_config2_repaired.urdf'))
    for geom in arm.geoms:
        geom.rgba = [.10, .42, .92, 1]
        geom.contype = geom.conaffinity = 0
    frame = scene.worldbody.add_frame(pos=[0, 0, .167], quat=[0, 1, 0, 0])
    scene.attach(arm, prefix='u_', frame=frame)
    model = scene.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = [f'u_joint{i}' for i in range(1, 7)] + ['u_trigger_joint']
    joints = [model.joint(n).id for n in names]
    zero_geom_pos = data.geom_xpos.copy()
    zero_geom_mat = data.geom_xmat.copy()
    anchors_parent = []
    axes_parent = []
    # Express each STEP-derived hinge anchor/axis in its stationary parent.
    for jid in joints:
        child = int(model.jnt_bodyid[jid]); parent = int(model.body_parentid[child])
        r = data.xmat[parent].reshape(3, 3)
        anchors_parent.append(r.T @ (data.xanchor[jid] - data.xpos[parent]))
        axes_parent.append(r.T @ data.xaxis[jid])
    base_geoms = [i for i in range(model.ngeom)
                  if model.body(int(model.geom_bodyid[i])).name == 'u_base_link']
    assert base_geoms, 'fixed base geometry missing'
    report = {'hardware_access': False, 'tests': [], 'physical_signs_verified': False}
    for index, jid in enumerate(joints):
        for degree in [-90, -45, 45, 90]:
            data.qpos[:] = 0
            data.qpos[int(model.jnt_qposadr[jid])] = math.radians(degree)
            mujoco.mj_forward(model, data)
            base_delta = float(np.max(np.abs(data.geom_xpos[base_geoms] - zero_geom_pos[base_geoms])))
            base_rotation_delta = float(np.max(np.abs(data.geom_xmat[base_geoms] - zero_geom_mat[base_geoms])))
            anchor_error = 0.0; axis_error = 0.0
            for k, tested in enumerate(joints):
                child = int(model.jnt_bodyid[tested]); parent = int(model.body_parentid[child])
                r = data.xmat[parent].reshape(3, 3)
                anchor_error = max(anchor_error, float(np.linalg.norm(
                    data.xanchor[tested] - (data.xpos[parent] + r @ anchors_parent[k]))))
                axis_error = max(axis_error, float(np.linalg.norm(data.xaxis[tested] - r @ axes_parent[k])))
            assert max(base_delta, base_rotation_delta, anchor_error, axis_error) < 1e-9
            report['tests'].append({'joint': names[index], 'degrees': degree,
                'base_translation_error_m': base_delta, 'base_rotation_error': base_rotation_delta,
                'hinge_anchor_error_m': anchor_error, 'hinge_axis_error': axis_error})
    # The geometric hinge checks complement independent STEP/mesh validation;
    # they do not establish encoder direction, physical joint limits or collision safety.
    size = 480
    model.vis.global_.offwidth = model.vis.global_.offheight = size
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = [-.08, -.03, .10]
    camera.distance = .70
    camera.azimuth = 40
    camera.elevation = -18
    outputs = []
    with mujoco.Renderer(model, height=size, width=size) as renderer:
        for degree in [45, -45]:
            sheet = Image.new('RGB', (size * 4, (size+32) * 2), '#131820')
            draw = ImageDraw.Draw(sheet)
            for tile in range(8):
                data.qpos[:] = 0
                if tile:
                    data.qpos[int(model.jnt_qposadr[joints[tile-1]])] = math.radians(degree)
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                x = tile % 4 * size; y = tile // 4 * (size+32)
                sheet.paste(Image.fromarray(renderer.render().copy()), (x, y+32))
                label = 'CAD assembly zero' if tile == 0 else f'{names[tile-1]} {degree:+d} deg'
                draw.text((x+12, y+10), label, fill='white')
            path = ROOT / 'logs' / f'uarm_repaired_output_axes_{degree:+d}.png'
            sheet.save(path); outputs.append(str(path))
    report['renders'] = outputs
    report['passed'] = True
    report['scope'] = '28 offline hinge sweeps; no real limits, collisions or encoder calibration certified'
    path = ROOT / 'logs/uarm_repaired_acceptance.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'passed':True,'sweeps':len(report['tests']),'report':str(path),'renders':outputs}))


if __name__ == '__main__': main()
