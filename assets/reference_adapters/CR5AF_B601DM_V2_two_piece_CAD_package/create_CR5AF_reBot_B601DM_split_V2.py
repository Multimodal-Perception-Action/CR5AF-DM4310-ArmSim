"""Parametric V2 prototype two-piece adapter: Dobot CR5AF flange to reBot B601-DM 4xM3 PCD38.
All units mm. This is a fit-test and access-verified concept; NOT certified to bear robot/grip loads.
Robot official interface: 4xM6 threaded PCD50 at 45deg; boss diameter 63, height 6.
Motor interface from user-provided measurement: 4 holes ~3mm on PCD38; thread/centre boss not verified.
Assembly: A -> robot via 4 M6; B -> motor via 4 M3 installed from B's bottom side;
then 4 M4 axial screws from B's gripper-facing outer perimeter into A's blind M4 holes.
"""
from pathlib import Path
import cadquery as cq, math, json
P = dict(
    base_od=82.0,
    base_height=16.0,
    base_robot_pilot_d=63.5,      # FDM fit trial, machine according to actual pilot
    base_robot_pilot_depth=6.25,
    base_robot_pcd=50.,
    base_robot_hole_d=6.6,
    base_robot_cb_d=11.3,
    base_robot_cb_depth=6.2,
    base_robot_phase=45.,
    base_join_pcd=68.,
    base_join_phase=0.,
    base_join_tap_d=3.3,       # CNC tapping drill for M4; NOT 3DP tap/heat insert bore
    base_join_tap_depth=7.5,
    tool_od=82.0,
    tool_height=8.0,
    tool_m3_pcd=38.,
    tool_m3_phase=45.,
    tool_m3_clear_d=3.4,
    tool_m3_cb_d=6.0,
    tool_m3_cb_depth=3.2,
    tool_join_clear_d=4.4,
    tool_join_cb_d=7.9,
    tool_join_cb_depth=4.3,
    tool_center_passage_d=12.,       # unverified; adjust when motor hub measured
    base_center_passage_d=12.,
    dowel_pcd=60.,
    dowel_d=3.1,                   # trial clearance, precision pin holes need CNC tolerancing
    dowel_phase=22.5,
    dowel_depth=3.2,
    debug_boss_relief=False,      # deliberately NOT guessing motor centre boss
)
O=Path('/mnt/data')
def cylinder(r,h,x=0,y=0,z=0):
    return cq.Solid.makeCylinder(r,h,cq.Vector(x,y,z))
def points(pcd,phase,count=4):
    return [(pcd/2*math.cos(math.radians(phase+i*360/count)),pcd/2*math.sin(math.radians(phase+i*360/count))) for i in range(count)]

def robot_plate():
    H=P['base_height']; S=cylinder(P['base_od']/2,H)
    # recess at z=0...6.25, open toward Dobot robot (robot 63mm boss protrudes toward +Z)
    S=S.cut(cylinder(P['base_robot_pilot_d']/2,P['base_robot_pilot_depth']+.015,z=-.01))
    for x,y in points(P['base_robot_pcd'],P['base_robot_phase']):
        S=S.cut(cylinder(P['base_robot_hole_d']/2,H+1,x,y,-.5))
        # M6 head sits below top of robot plate; tool plate can then sit flush on top
        S=S.cut(cylinder(P['base_robot_cb_d']/2,P['base_robot_cb_depth']+.01,x,y,H-P['base_robot_cb_depth']))
    for x,y in points(P['base_join_pcd'],P['base_join_phase']):
        # M4 blind TAP from top, threaded later on CNC part; STEP shows tap-drill cylinder
        S=S.cut(cylinder(P['base_join_tap_d']/2,P['base_join_tap_depth']+.01,x,y,H-P['base_join_tap_depth']))
    for x,y in points(P['dowel_pcd'],P['dowel_phase'],2):
        S=S.cut(cylinder(P['dowel_d']/2,P['dowel_depth']+.01,x,y,H-P['dowel_depth']))
    S=S.cut(cylinder(P['base_center_passage_d']/2,H+1,z=-.5))
    return S.clean()

def tool_plate():
    # Local z = 0 is the bottom that will contact the robot plate top.
    # Its M3 head counterbores are on the BOTTOM (reachable BEFORE two plates are bolted together).
    H=P['tool_height'];S=cylinder(P['tool_od']/2,H)
    S=S.cut(cylinder(P['tool_center_passage_d']/2,H+1,z=-.5))
    for x,y in points(P['tool_m3_pcd'],P['tool_m3_phase']):
        S=S.cut(cylinder(P['tool_m3_clear_d']/2,H+1,x,y,-.5))
        S=S.cut(cylinder(P['tool_m3_cb_d']/2,P['tool_m3_cb_depth']+.01,x,y,-.005))
    for x,y in points(P['base_join_pcd'],P['base_join_phase']):
        S=S.cut(cylinder(P['tool_join_clear_d']/2,H+1,x,y,-.5))
        # M4 head accessible from tool/motor side at r=34, motor must not block driver.
        S=S.cut(cylinder(P['tool_join_cb_d']/2,P['tool_join_cb_depth']+.01,x,y,H-P['tool_join_cb_depth']))
    for x,y in points(P['dowel_pcd'],P['dowel_phase'],2):
        S=S.cut(cylinder(P['dowel_d']/2,P['dowel_depth']+.01,x,y,-.005))
    return S.clean()

A=robot_plate();B=tool_plate()
for k,obj in [('Robot_Plate_A',A),('Gripper_Plate_B',B)]:
    assert obj.isValid() and len(obj.Solids())==1, (k,obj.isValid(),len(obj.Solids()))
    cq.exporters.export(obj,str(O/f'CR5AF_B601DM_V2_{k}.step'),exportType='STEP')
    cq.exporters.export(obj,str(O/f'CR5AF_B601DM_V2_{k}.stl'),exportType='STL',tolerance=.1,angularTolerance=.12)
    re=cq.importers.importStep(str(O/f'CR5AF_B601DM_V2_{k}.step')).val()
    print(k,'valid',re.isValid(),'volume',round(re.Volume(),1),'bb',tuple(round(d,2) for d in [re.BoundingBox().xlen,re.BoundingBox().ylen,re.BoundingBox().zlen]))
# Save multi-body assembly at assembled z offsets (not fused, individual parts stay separate)
Bpos=B.translate(cq.Vector(0,0,P['base_height']))
assy=cq.Assembly(name='CR5AF_B601DM_Split_Adapter_V2')
assy.add(A,name='Robot_A',color=cq.Color(.55,.61,.69))
assy.add(Bpos,name='Gripper_B',color=cq.Color(.16,.62,.69))
assy.save(str(O/'CR5AF_B601DM_V2_two_piece_ASSEMBLY.step'))
# Exploded assembly B above A by 11mm
exploded=cq.Assembly(name='CR5AF_B601DM_Exploded_V2')
exploded.add(A,name='Robot_A',color=cq.Color(.55,.61,.69))
exploded.add(Bpos.translate(cq.Vector(0,0,11)),name='Gripper_B',color=cq.Color(.16,.62,.69))
exploded.save(str(O/'CR5AF_B601DM_V2_EXPLODED.step'))
print('M6x10 screw effective engagement, given head underside at',round(P['base_height']-P['base_robot_cb_depth'],2),'from robot side, and robot pilot recess ceiling',P['base_robot_pilot_depth'],':',round(10-((P['base_height']-P['base_robot_cb_depth'])-P['base_robot_pilot_depth']),2),'mm')
print('M4x10 joining screw effective depth:', round(10-(P['tool_height']-P['tool_join_cb_depth']),2),'mm')
print('A->B positioning dowels Ø3 x ~6mm: install holes 3.2 deep each; allow 0.4mm end slack')
# shape summary to inspect and help edit
(O/'CR5AF_B601DM_V2_parameters.json').write_text(json.dumps(P,indent=2,ensure_ascii=False),encoding='utf8')
