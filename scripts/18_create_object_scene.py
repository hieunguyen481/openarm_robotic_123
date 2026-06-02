"""Create v2/cell_object.xml by adding a target cube to the cell scene."""

from pathlib import Path
import xml.etree.ElementTree as ET

from stage1_common import ROOT

SRC_XML = ROOT / "v2" / "cell.xml"
DST_XML = ROOT / "v2" / "cell_object.xml"

tree = ET.parse(SRC_XML)
root = tree.getroot()

worldbody = root.find("worldbody")
if worldbody is None:
    raise RuntimeError("Cannot find <worldbody> in XML")

for body in list(worldbody.findall("body")):
    if body.attrib.get("name") == "target_cube":
        worldbody.remove(body)

cube = ET.SubElement(
    worldbody,
    "body",
    {
        "name": "target_cube",
        "pos": "0.42 0.19 1.035",
    },
)
ET.SubElement(cube, "freejoint", {"name": "target_cube_freejoint"})
ET.SubElement(
    cube,
    "geom",
    {
        "name": "target_cube_geom",
        "type": "box",
        "size": "0.025 0.025 0.025",
        "mass": "0.05",
        "rgba": "1 0.2 0.2 1",
        "friction": "1.0 0.005 0.0001",
    },
)

Path(DST_XML).parent.mkdir(exist_ok=True)
tree.write(DST_XML, encoding="utf-8", xml_declaration=True)
print(f"Saved object scene to: {DST_XML}")
