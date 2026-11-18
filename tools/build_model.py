"""Build the offline V2R1 model from the exact November 16 STEP export."""
import argparse
from collections import defaultdict
from io import BytesIO
from hashlib import sha256
from pathlib import Path
from shutil import copyfile
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label

tool_dir = Path(__file__).resolve().parent
sources = tool_dir.parent / "models"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("step", type=Path, help="local 2026-11-16 V2R1 STEP or sanitized copy")
parser.add_argument("--output", type=Path, default=Path("generated") / "cr5af_v2r1_x3")
args = parser.parse_args()
if not args.step.is_file():
    parser.error("the supplied STEP path is not a file")
step_bytes = args.step.read_bytes()
_, separator, step_data = step_bytes.partition(b"DATA;")
expected_sha = "7da40da4f34703ced70767e458dd1ea6a9ca5ae24705e2c5081788a675b0e1eb"
if not separator or sha256(step_data).hexdigest() != expected_sha:
    parser.error("the STEP data does not match the 2026-11-16 V2R1 export")
out = args.output.resolve()
if out.exists():
    parser.error("--output must be a new directory")
geom = out / "meshes/v2r1"
geom.mkdir(exist_ok=True, parents=True)
for src in (sources / "dobot_rviz/meshes/cr5af").glob("*.STL"):
    dest = out / "meshes/cr5af" / src.name
    dest.parent.mkdir(exist_ok=True)
    copyfile(src, dest)
copyfile(sources / "LICENSE_DOBOT_MIT", out / "LICENSE_DOBOT_MIT")

doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
if reader.ReadStream("local_v2r1.step", BytesIO(step_bytes)) != IFSelect_RetDone or not reader.Transfer(doc):
    raise RuntimeError("11/16 V2R1 STEP failed to load.")
shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = Sequence_TDF_Label()
shape_tool.GetFreeShapes(roots)
assert roots.Length() == 1
occurrences = Sequence_TDF_Label()
shape_tool.GetComponents_s(roots.Value(1), occurrences)
assert occurrences.Length() == 88

def label_name(label):
    attr = TDataStd_Name()
    assert label.FindAttribute(TDataStd_Name.GetID_s(), attr)
    return attr.Get().ToExtString()

positive = {11,15,16,17,20,21,22,48,49,56}
negative = {14,18,19,23,24,25,47,52,53,55}
camera_main = {63,64,65,68,69,70,87}
camera_mount = {60,61,62}
cos45 = math.sqrt(0.5)
# This is the CAD mechanical camera frame, taken from the mount datum and camera body bottom face.
# Rz(pi) is applied after Rx(pi/4) in the URDF RPY convention.
camera_R = np.array([[-1.,0.,0.],[0.,-cos45,cos45],[0.,cos45,cos45]])
camera_t_mm = np.array([0.,72.,-15.])
dedicated = {7: "motor", 57: "direct_base", 58: "fuse_cartridge",
             59: "retaining_pin", 88: "tether_route", 66: "x3_lenses", 67: "x3_screen"}
rotating_preview = {9,12,40,51}
rail_parts = {10,50,54}
fingers = {55: "jaw_negative_tip", 56: "jaw_positive_tip"}

def category(name):
    if name in fingers:
        return fingers[name]
    if name in dedicated:
        return dedicated[name]
    if name in positive:
        return "jaw_positive_mechanism"
    if name in negative:
        return "jaw_negative_mechanism"
    if name in camera_main:
        return "x3_body"
    if name in camera_mount:
        return "x3_bracket"
    if name in rotating_preview:
        return "pinion"
    if name in rail_parts:
        return "fixed_rail_mounts"
    return "fixed_hardware"

def as_triangles(shape):
    tess = BRepMesh_IncrementalMesh(shape, 0.22, False, 0.28, True)
    if not tess.IsDone():
        raise RuntimeError("Meshing did not finish")
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        arr = BRep_Tool.Triangulation_s(face, loc)
        if arr is not None:
            affine = loc.Transformation()
            nodes = []
            for j in range(1, arr.NbNodes()+1):
                p = arr.Node(j).Transformed(affine)
                nodes.append((p.X(), p.Y(), p.Z()))
            flip = face.Orientation() == TopAbs_REVERSED
            for k in range(1, arr.NbTriangles()+1):
                a,b,c = arr.Triangle(k).Get()
                if flip:
                    b,c = c,b
                yield nodes[a-1], nodes[b-1], nodes[c-1]
        ex.Next()

groups = defaultdict(list)
membership = {}
for i in range(1,occurrences.Length()+1):
    ref = TDF_Label()
    occurrence = occurrences.Value(i)
    assert shape_tool.GetReferredShape_s(occurrence, ref)
    shape = shape_tool.GetShape_s(occurrence)
    assert not shape.IsNull()
    name = label_name(ref)
    assert name and (not name.startswith("G_") or name.startswith(f"G_{i:03}_"))
    mesh = category(i)
    if name in membership:
        raise RuntimeError("Duplicate STEP occurrence name")
    membership[name] = mesh
    tris = list(as_triangles(shape))
    if not tris:
        raise RuntimeError(f"No tessellation for {name}")
    groups[mesh].extend(tris)
    if i % 15 == 0:
        print(f"Meshed {i}/88 STEP occurrences", flush=True)

def write_obj(name, triangles):
    to_camera = name in {"x3_body","x3_screen","x3_lenses"}
    vertices = {}
    faces = []
    arr = np.array(triangles)
    if to_camera:
        arr = (arr - camera_t_mm) @ camera_R
    arr = np.round(arr * 0.001, 8)
    for tri in arr:
        coords = [tuple(x) for x in tri]
        if len(set(coords)) != 3:
            continue
        indices = [vertices.setdefault(p,len(vertices)+1) for p in coords]
        faces.append(indices)
    p = geom / f"{name}.obj"
    with p.open("w",encoding="ascii",newline="\n") as fh:
        fh.write("# V2R1 November 16 STEP. Coordinates in meters.\n")
        for x,y,z in vertices:
            fh.write(f"v {x:.8f} {y:.8f} {z:.8f}\n")
        for a,b,c in faces:
            fh.write(f"f {a} {b} {c}\n")
    return {"vertices":len(vertices),"triangles":len(faces)}

meshes = {name:write_obj(name,triangles) for name,triangles in sorted(groups.items())}
print(f"Exported {len(meshes)} mesh groups",flush=True)
model = ET.parse(sources / "cr5af_portable.urdf")
root = model.getroot()
root.set("name","CR5AF_V2R1_DM4310_X3")
for tag in root.findall(".//mesh"):
    fname = tag.attrib["filename"].rsplit("/",1)[1]
    tag.set("filename","meshes/cr5af/"+fname)
mujoco = ET.Element("mujoco")
ET.SubElement(mujoco, "compiler", discardvisual="false", fusestatic="false", strippath="false", balanceinertia="true")
root.insert(0,mujoco)

def add(parent,tag,**attrs):
    return ET.SubElement(parent,tag,attrs)

def origin(parent,xyz="0 0 0",rpy="0 0 0"):
    return add(parent,"origin",xyz=xyz,rpy=rpy)

materials = {
    "alloy":"0.64 0.65 0.68 1", "motor_dark":"0.26 0.27 0.31 1",
    "orange":"0.79 0.49 0.16 1","fuse":"0.83 0.27 0.13 1",
    "teal":"0.05 0.44 0.46 1","camera_black":"0.11 0.11 0.13 1",
    "camera_glass":"0.17 0.23 0.31 1","wire":"0.85 0.25 0.10 1",
}
for k,rgba in materials.items():
    add(add(root,"material",name="v2r1_"+k),"color",rgba=rgba)

def link(name,mass=None,xyz=None,diagonal=None):
    ln=add(root,"link",name=name)
    if mass is not None:
        inert=add(ln,"inertial")
        origin(inert,xyz)
        add(inert,"mass",value=str(mass))
        add(inert,"inertia",ixx=str(diagonal[0]),iyy=str(diagonal[1]),izz=str(diagonal[2]),
            ixy="0",ixz="0",iyz="0")
    return ln

def mesh(link,kind,name,material=None):
    target=add(link,kind,name=name if kind == "visual" else f"{name}_collision")
    add(add(target,"geometry"),"mesh",filename=f"meshes/v2r1/{name}.obj")
    if material:
        add(target,"material",name="v2r1_"+material)

def joint(name,ptype,parent,child,xyz="0 0 0",rpy="0 0 0"):
    jt=add(root,"joint",name=name,type=ptype)
    origin(jt,xyz,rpy)
    add(jt,"parent",link=parent)
    add(jt,"child",link=child)
    return jt

flange=link("v2r1_flange_F",0.92,"0 0.016 0.059",(0.0014,0.0013,0.0010))
joint("v2r1_flange_mount","fixed","Link6","v2r1_flange_F","0 0.0492 0","-1.570796326794897 0 0")
for name,color in (
    ("direct_base","orange"),("motor","motor_dark"),("fixed_rail_mounts","alloy"),
    ("fixed_hardware","alloy"),("fuse_cartridge","fuse"),("retaining_pin","fuse"),
    ("tether_route","wire"),("pinion","motor_dark")
):
    mesh(flange,"visual",name,color)
for name in ("direct_base","motor"):
    mesh(flange,"collision",name)

bracket=link("x3_camera_bracket",0.085,"0 0.088 0.022",(0.00013,0.00012,0.00010))
joint("x3_bracket_mount","fixed","v2r1_flange_F","x3_camera_bracket")
mesh(bracket,"visual","x3_bracket","teal")
mesh(bracket,"collision","x3_bracket")

camera=link("x3_cad_body",0.18,"0 0 0.057",(0.00023,0.00022,0.000045))
joint("x3_cad_body_mount","fixed","x3_camera_bracket","x3_cad_body","0 0.072 -0.015",
      "0.7853981633974483 0 3.141592653589793")
for name,color in (("x3_body","camera_black"),("x3_lenses","camera_glass"),("x3_screen","camera_glass")):
    mesh(camera,"visual",name,color)
mesh(camera,"collision","x3_body")
# Perspective preview extrinsics derived from the CAD lens envelope, for virtual cameras only.
for tag,sign,rotation in (
    ("positive_y",1,"-1.570796326794897 0 0"),
    ("negative_y",-1,"-1.570796326794897 0 3.141592653589793"),
):
    optical = f"x3_preview_{tag}_optical"
    link(optical)
    joint(f"{optical}_mount","fixed","x3_cad_body",optical,f"0 {sign*0.0154} 0.09302",rotation)

for name,sign in (("positive",1),("negative",-1)):
    side=link(f"jaw_{name}",0.10,
              f"{sign*0.024} {sign*0.024} 0.125",(0.000095,0.000095,0.000041))
    jt=joint(f"jaw_{name}_slide","prismatic","v2r1_flange_F",f"jaw_{name}")
    add(jt,"axis",xyz=f"{sign*cos45:.16g} {sign*cos45:.16g} 0")
    add(jt,"limit",lower="-0.0075",upper="0.0125",effort="8",velocity="0.035")
    add(jt,"dynamics",damping="1.5",friction="0")
    if sign < 0:
        add(jt,"mimic",joint="jaw_positive_slide",multiplier="1",offset="0")
    for group in ("tip","mechanism"):
        mesh(side,"visual",f"jaw_{name}_{group}","alloy")
    mesh(side,"collision",f"jaw_{name}_tip")

ET.indent(model,space="  ")
model.write(out/"cr5af_v2r1_x3.urdf",encoding="utf-8",xml_declaration=True)
provenance = {
    "source_step": "V2R1_END_EFFECTOR_COMPLETE_F_FRAME.stp",
    "source_step_data_sha256": expected_sha,
    "step_export_date": "2026-11-16",
    "source_occurrence_count": len(membership),
    "arm_source_commit": "7807ef8fb31e8f86a76cd474a4a9cddffef6eca9",
    "coordinates": "Geometry in meters. STEP CAD datums. CAD camera frame is mechanical.",
    "geometry_groups": meshes,
    "source_occurrence_to_mesh": membership,
}
(out/"mesh_provenance.json").write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")

copyfile(tool_dir / "preview.py", out / "preview.py")
copyfile(tool_dir / "NOTICE_GENERATED.md", out / "NOTICE_GENERATED.md")
