"""Offline STEP entity audit: output gear axes, independent of prior URDF."""
from pathlib import Path
import re
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STEP = ROOT / 'uarm_cr10a_teleop_sim/LeRobot-Anything-U-Arm-main/mechanical/Feetech_servo/Config2_STL/config2.STEP'


class Step:
    def __init__(self):
        self.entities = {int(k): v.strip() for k, v in re.findall(r'#(\d+)\s*=\s*(.*?);', STEP.read_text(errors='replace'), re.S)}
        self.reverse = {}
        for i in self.entities:
            for j in self.refs(i): self.reverse.setdefault(j, []).append(i)
        products = {i: re.findall(r"'([^']*)'", v)[0] for i, v in self.entities.items() if v.startswith('PRODUCT (')}
        self.names = {}
        for i, v in self.entities.items():
            if v.startswith('PRODUCT_DEFINITION ('):
                for f in self.refs(i):
                    if self.entities[f].startswith('PRODUCT_DEFINITION_FORMATION'):
                        for p in self.refs(f):
                            if p in products: self.names[i] = products[p]
        self.children = {}
        for i,v in self.entities.items():
            if v.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
                parent, child = self.refs(i)
                label = re.findall(r"'([^']*)'", v)[0]
                pds = next(x for x in self.reverse[i] if self.entities[x].startswith('PRODUCT_DEFINITION_SHAPE'))
                context = next(x for x in self.reverse[pds] if self.entities[x].startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'))
                relation = self.refs(context)[0]
                tr = next(x for x in self.refs(relation) if self.entities[x].startswith('ITEM_DEFINED_TRANSFORMATION'))
                a,b = self.refs(tr)
                transform = self.frame(a) @ np.linalg.inv(self.frame(b))
                self.children.setdefault(parent, []).append((label,child,transform,i))

    def refs(self,i): return list(map(int,re.findall(r'#(\d+)', self.entities[i])))

    def vector(self,i):
        numbers = self.entities[i].split('(', 2)[-1].replace(')', '').split(',')
        return np.array([float(v.strip()) for v in numbers])

    def frame(self,i):
        refs=self.refs(i); p=self.vector(refs[0]); z=self.vector(refs[1]); x=self.vector(refs[2]); y=np.cross(z,x)
        T=np.eye(4);T[:3,:3]=np.column_stack([x,y,z]);T[:3,3]=p;return T

    def walk(self,pd,transform=None,path=()):
        if transform is None: transform=np.eye(4)
        yield pd,transform,path
        for label,child,T,_ in self.children.get(pd,[]):
            yield from self.walk(child,transform@T,path+(label,))


def audit():
    step=Step(); rows=[]
    old=json.loads((ROOT/'uarm_cr10a_teleop_sim/uarm_cr10a_teleop_sim/config/kinematic_verification.json').read_text())['uarm_config2']
    old_axes={e['instance']:e for e in old['servo_axis_evidence']}
    old_axes['NAUO14']=old['gripper_servo_axis']
    roots=['NAUO4','NAUO5','NAUO6','NAUO8','NAUO11','NAUO12','NAUO14']
    nodes=list(step.walk(114529))
    for index,inst in enumerate(roots,1):
        gear=next((T,path) for pd,T,path in nodes if path[0:1]==(inst,) and step.names[pd]=='OUTPUTGEAR_20.stp')
        motor=next((T,path) for pd,T,path in nodes if path[0:1]==(inst,) and step.names[pd]=='MOTOR-1723_3_1.stp')
        T,path=gear; p=T[:3,3]; axis=T[:3,2]; motor_p=motor[0][:3,3]; motor_axis=motor[0][:3,1]
        old_p=np.array(old_axes[inst]['point_mm']); old_axis=np.array(old_axes[inst]['axis'])
        offset=np.linalg.norm(np.cross(p-old_p,axis)); motor_error=np.linalg.norm(np.cross(motor_p-old_p,motor_axis))
        rows.append({'joint':index,'instance':inst,'output_gear_path':path,'output_gear_frame_mm':T.tolist(),'output_axis_point_mm':p.tolist(),'output_axis_direction':axis.tolist(),'old_motor_point_mm':old_p.tolist(),'old_axis_to_correct_axis_distance_mm':float(offset),'old_axis_vs_motor_axis_distance_mm':float(motor_error),'axis_parallel_dot_abs':float(abs(axis@old_axis)),'motor_axis_point_mm':motor_p.tolist(),'motor_axis_direction':motor_axis.tolist()})
    report={'source_step':str(STEP),'method':'STEP NEXT_ASSEMBLY_USAGE_OCCURRENCE and ITEM_DEFINED_TRANSFORMATION chains; OUTPUTGEAR local Z axis from R2.95 and R3 cylinders','old_axis_error':'R8.525 cylinders are MOTOR-1723_3_1 motor housing surfaces, not the OUTPUTGEAR shaft','axes':rows,'hardware_access':False}
    out=ROOT/'simulation/generated/uarm_repaired/step_axis_evidence.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    for r in rows: print(r['joint'],r['instance'],'p',r['output_axis_point_mm'],'a',r['output_axis_direction'],'offset',r['old_axis_to_correct_axis_distance_mm'],'motor_error',r['old_axis_vs_motor_axis_distance_mm'])
    return report


if __name__=='__main__':audit()
