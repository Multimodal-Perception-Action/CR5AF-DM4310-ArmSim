"""Offline repair of Config2 link ownership; preserves the original package."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'models/source/uarm_cr10a_teleop_sim'
if not PACKAGE.is_dir():
    PACKAGE = ROOT / 'uarm_cr10a_teleop_sim/uarm_cr10a_teleop_sim'
SOURCE = PACKAGE / 'models/urdf/uarm_config2_kinematic.urdf'
OFFICIAL = ROOT / 'models/source/LeRobot-Anything-U-Arm/mechanical/Feetech_servo/Config2_STL'
if not OFFICIAL.is_dir():
    OFFICIAL = ROOT / 'uarm_cr10a_teleop_sim/LeRobot-Anything-U-Arm-main/mechanical/Feetech_servo/Config2_STL'
DEST = ROOT / 'simulation/generated/uarm_repaired'
SHIFT = np.array([-.14275045585632323, -.12785960578918457, -.05297554016113282])


def build():
    tree = ET.parse(SOURCE); root = tree.getroot()
    frames = {'base_link': np.eye(4)}
    for j in root.findall('joint'):
        o = j.find('origin'); t = np.eye(4)
        t[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        frames[j.find('child').get('link')] = frames[j.find('parent').get('link')] @ t
    meshes = {}
    for link in root.findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is None: continue
        m = trimesh.load_mesh((SOURCE.parent / mesh.get('filename')).resolve(), process=True)
        m.apply_scale(.001); meshes[link.get('name')] = m
    pieces = list(meshes['link6'].split(only_watertight=False, repair=False))
    selected = {}; evidence = {}
    for name in ['sitter-1', 'sitter-2', 'sitter_mid-1', 'trigger_short-1']:
        ref = trimesh.load_mesh(OFFICIAL / f'config2 - {name}.STL', process=False)
        ref.apply_scale(.001); ref.apply_translation(SHIFT)
        candidates = []
        for index, piece in enumerate(pieces):
            if index in selected.values(): continue
            w = piece.copy(); w.apply_transform(frames['link6'])
            error = float(np.max(np.abs(w.bounds - ref.bounds)))
            candidates.append((error, index))
        error, idx = min(candidates)
        if error > .0002:
            raise RuntimeError(f'{name}: component bounds mismatch {error} m')
        selected[name] = idx
        evidence[name] = {'old_link': 'link6', 'new_link': 'trigger' if name.startswith('trigger') else 'base_link', 'bounds_error_m': error, 'faces': len(pieces[idx].faces)}
    fixed = []
    for name, idx in selected.items():
        if name.startswith('trigger'): continue
        p = pieces[idx].copy(); p.apply_transform(frames['link6']); fixed.append(p)
    meshes['base_link'] = trimesh.util.concatenate([meshes['base_link']] + fixed)
    meshes['link6'] = trimesh.util.concatenate([p for i, p in enumerate(pieces) if i not in selected.values()])
    trigger = pieces[selected['trigger_short-1']].copy()
    verification = json.loads((PACKAGE / 'config/kinematic_verification.json').read_text(encoding='utf-8'))['uarm_config2']
    tool = np.array(verification['tool_frame_world_mm'], dtype=float); tool[:3, 3] *= .001
    axis_report = json.loads((DEST / 'step_axis_evidence.json').read_text(encoding='utf-8'))
    # Split the moving output gear/shaft and the front/rear servo horns from
    # the stationary servo housing. Classify by their STEP output-frame bounds,
    # not only by triangle counts (which can differ between tessellations).
    rotor_bounds_mm = {
        'gear_disc': [[-8.66,-8.66,-6.5],[8.66,8.66,0]],
        'splined_shaft': [[-3,-3,-13.6],[3,3,-2.8]],
        'rear_shaft': [[-3,-3,-2.3],[3,3,7.8]],
        'front_horn': [[-9.975,-9.975,-14.5],[9.975,9.975,-10]],
        'rear_horn': [[-9.975,-9.975,19],[9.975,9.975,22.1]],
    }
    sources = {k:v.copy() for k,v in meshes.items()}
    removals = {}; transfers = {k:[] for k in meshes}; trigger_parts = []
    rotor_evidence = []
    for entry in axis_report['axes']:
        idx = entry['joint']; parent = 'base_link' if idx==1 else f'link{idx-1}'
        child = f'link{idx}' if idx<=6 else 'trigger'
        output_frame = np.array(entry['output_gear_frame_mm']); output_frame[:3,3] *= .001
        comps = list(sources[parent].split(only_watertight=False, repair=False)); chosen = set()
        for part, bounds in rotor_bounds_mm.items():
            matches=[]
            for ci,comp in enumerate(comps):
                if ci in chosen: continue
                candidate=comp.copy(); candidate.apply_transform(np.linalg.inv(output_frame)@frames[parent])
                err=float(np.max(np.abs(candidate.bounds*1000-np.array(bounds))))
                matches.append((err,ci))
            err,ci=min(matches)
            if err > .15: raise RuntimeError(f'J{idx} {part} split mismatch {err} mm')
            chosen.add(ci); part_mesh=comps[ci].copy()
            if idx<=6:
                part_mesh.apply_transform(np.linalg.inv(frames[child])@frames[parent]);transfers[child].append(part_mesh)
            else: trigger_parts.append(part_mesh)
            rotor_evidence.append({'joint':idx,'part':part,'bounds_error_mm':err,'faces':len(comps[ci].faces),'from':parent,'to':child})
        removals[parent] = trimesh.util.concatenate([c for ci,c in enumerate(comps) if ci not in chosen])
    meshes = {name:trimesh.util.concatenate([removals.get(name,sources[name])]+transfers[name]) for name in sources}
    trigger = trimesh.util.concatenate([trigger]+trigger_parts)
    corrected = {k: v.copy() for k, v in frames.items()}
    for entry in axis_report['axes']:
        index = entry['joint']
        if index <= 6:
            key = f'link{index}'
            # Keep the existing geometric sign convention, but use the true
            # output shaft centre, not the internal motor axis 25.53 mm away.
            z = np.array(entry['output_axis_direction'])
            if abs(float(z @ corrected[key][:3, 2])) < .999999:
                raise RuntimeError(f'J{index} axis is not parallel to verified output shaft')
            corrected[key][:3, 3] = np.array(entry['output_axis_point_mm']) * .001
        else:
            tool[:3, 3] = np.array(entry['output_axis_point_mm']) * .001
    for name in list(meshes):
        meshes[name].apply_transform(np.linalg.inv(corrected[name]) @ frames[name])
    # Trigger mesh is still in the OLD link6-local frame here.
    trigger.apply_transform(np.linalg.inv(tool) @ frames['link6'])
    meshes['trigger'] = trigger
    corrected['tool0'] = tool
    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link'); child = joint.find('child').get('link')
        relative = np.linalg.inv(corrected[parent]) @ corrected[child]
        origin = joint.find('origin')
        origin.set('xyz', ' '.join(map(str, relative[:3, 3])))
        origin.set('rpy', ' '.join(map(str, Rotation.from_matrix(relative[:3, :3]).as_euler('xyz'))))
    rel = np.linalg.inv(corrected['link6']) @ tool
    DEST.mkdir(parents=True, exist_ok=True)
    for name, mesh in meshes.items():
        mesh.export(DEST / f'{name}.stl')
    for link in root.findall('link'):
        for mesh in link.findall('./visual/geometry/mesh') + link.findall('./collision/geometry/mesh'):
            mesh.set('filename', f"{link.get('name')}.stl"); mesh.set('scale', '1 1 1')
    link = ET.SubElement(root, 'link', name='trigger')
    for kind in ['visual', 'collision']:
        ET.SubElement(ET.SubElement(ET.SubElement(link, kind), 'geometry'), 'mesh', filename='trigger.stl', scale='1 1 1')
    j = ET.SubElement(root, 'joint', name='trigger_joint', type='continuous')
    ET.SubElement(j, 'parent', link='link6'); ET.SubElement(j, 'child', link='trigger')
    ET.SubElement(j, 'origin', xyz=' '.join(map(str, rel[:3, 3])), rpy=' '.join(map(str, Rotation.from_matrix(rel[:3, :3]).as_euler('xyz'))))
    ET.SubElement(j, 'axis', xyz='0 0 1'); ET.SubElement(j, 'limit', effort='1', velocity='6.28318531')
    root.set('name', 'uarm_config2_repaired_visual')
    ET.indent(tree, space='  '); tree.write(DEST / 'uarm_config2_repaired.urdf', encoding='utf-8', xml_declaration=True)
    report = {'source': str(SOURCE), 'output': str(DEST), 'part_reassignment': evidence, 'rotating_part_reassignment':rotor_evidence, 'axis_source': 'step_axis_evidence.json', 'all_seven_old_axis_offsets_mm': [e['old_axis_to_correct_axis_distance_mm'] for e in axis_report['axes']], 'hardware_access': False, 'directions_and_physical_zero': 'require separate visual registration'}
    (DEST / 'repair_evidence.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__': build()
