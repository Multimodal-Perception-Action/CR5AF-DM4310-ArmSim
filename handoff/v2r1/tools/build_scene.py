import argparse
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import mujoco

parser = argparse.ArgumentParser(description="Build the MuJoCo scene from a generated V2R1 URDF.")
parser.add_argument("directory", type=Path, help="output directory from build_model.py")
root = parser.parse_args().directory.resolve()
urdf = root / "cr5af_v2r1_x3.urdf"
if not urdf.is_file():
    parser.error("the generated URDF is missing")
dest = root / "cr5af_v2r1_x3.mjcf.xml"
if dest.exists():
    parser.error("the scene already exists")
arm = [f"joint{i}" for i in range(1,7)]
model = mujoco.MjModel.from_xml_path(str(urdf))
with TemporaryDirectory(prefix="cr5af_v2r1_") as staging:
    compiled = Path(staging) / "compiled.xml"
    mujoco.mj_saveLastXML(str(compiled), model)
    tree = ET.parse(compiled)
doc = tree.getroot()
body = doc.find(".//body[@name='x3_cad_body']")
assert body is not None
for label,sign in (("positive_y",1),("negative_y",-1)):
    ET.SubElement(body,"camera",name=f"x3_preview_{label}",
                  pos=f"0 {sign*0.0154} 0.09302",
                  xyaxes=f"{sign} 0 0 0 0 1",fovy="90")

options = ET.Element("option",timestep="0.002",integrator="implicitfast")
doc.insert(1,options)
# The vendor base and first-link collision hulls overlap at the physical bearing.
contact=ET.SubElement(doc,"contact")
ET.SubElement(contact,"exclude",name="base_j1_bearing",body1="base_link",body2="Link1")
equality=ET.SubElement(doc,"equality")
ET.SubElement(equality,"joint",name="symmetric_jaws",
              joint1="jaw_negative_slide",joint2="jaw_positive_slide",
              polycoef="0 1 0 0 0",solref="0.006 1")
actuators=ET.SubElement(doc,"actuator")
for i,name in enumerate(arm):
    jt=doc.find(f".//joint[@name='{name}']")
    jt.set("damping",("8","8","8","4","4","2")[i])
    ET.SubElement(actuators,"position",name=f"{name}_target",joint=name,
                  kp=("800","1300","1000","350","350","180")[i],kv=("44","70","55","24","24","14")[i],
                  ctrlrange=jt.get("range"),forcerange="-100 100")
ET.SubElement(actuators,"position",name="jaw_per_side_travel_m",joint="jaw_positive_slide",
              kp="1400",kv="15",ctrlrange="-0.0075 0.0125",forcerange="-8 8")
wb=doc.find("worldbody")
# Use two diffuse lights to display the CAD surfaces without changing simulation geometry.
ET.SubElement(wb,"light",pos="0 -1 2.5",dir="0 0.35 -1",diffuse="0.85 0.85 0.85")
ET.SubElement(wb,"light",pos="1.4 1.2 1.4",dir="-1 -1 -0.6",diffuse="0.35 0.35 0.35")
for geom in doc.findall(".//geom"):
    if geom.get("contype") != "0":
        geom.set("group","3")
ET.indent(tree,space="  ")
tree.write(dest,encoding="utf-8",xml_declaration=True)
