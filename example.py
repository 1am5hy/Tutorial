import xml.etree.ElementTree as ET

root = ET.Element("mujoco", model="example")
doc = ET.SubElement(root, "compiler", coordinate="global")
doc1 = ET.SubElement(root, "default")
doc12 = ET.SubElement(doc1, "geom", rgba=".8 .6 .4 1")

doc2 = ET.SubElement(root, "asset")
doc21 = ET.SubElement(doc2, "texture", type="skybox", builtin="gradient", rgb2=".6 .8 1",
                      width="256", height="256")

doc3 = ET.SubElement(root, "worldbody")
doc31 = ET.SubElement(doc3, "light", pos="0 1 1", dir='0 -1 -1', diffuse="1 1 1")
doc32 = ET.SubElement(doc3, "body")
doc321 = ET.SubElement(doc32, "geom", type="capsule", fromto="0 0 1  0 0 0.6", size="0.06")
doc322 = ET.SubElement(doc32, "joint", type="ball", pos="0 0 1")
doc323 = ET.SubElement(doc32, "body")
doc3231 = ET.SubElement(doc323, "geom", type="capsule", fromto="0 0 0.6  0.3 0 0.6", size="0.04")
doc3232 = ET.SubElement(doc323, "joint", type="hinge", pos="0 0 0.6", axis="0 1 0")
doc3233 = ET.SubElement(doc323, "joint", type="hinge", pos="0 0 0.6", axis="0 0 1")
doc3234 = ET.SubElement(doc323, "body")
doc32341 = ET.SubElement(doc3234, "geom", type="ellipsoid", pos="0.4 0 0.6", size="0.1 0.08 0.02")
doc32342 = ET.SubElement(doc3234, "site", name="end1", pos="0.5 0 0.6", tpe="sphere", size="0.01")
doc32343 = ET.SubElement(doc3234, "joint", type="hinge", pos="0.3 0 0.6", axis="0 0 1")
doc32344 = ET.SubElement(doc3234, "joint", type="hinge", pos="0.3 0 0.6", axis="0 0 1")

doc33 = ET.SubElement(doc3, "body")
doc331 = ET.SubElement(doc33, "geom", type="cylinder", fromto="0.5 0 0.2  0.5 0 0", size="0.07")
doc332 = ET.SubElement(doc33, "site", name="end2", pos="0.5 0 0.2", type="sphere", size="0.01")
doc333 = ET.SubElement(doc33, "joint", type="free")

doc4 = ET.SubElement(root, "tendon")
doc41 = ET.SubElement(doc4, "spatial", limited="true", range="0 0.6", width="0.005")
doc411 = ET.SubElement(doc41, "site", site="end1")
doc412 = ET.SubElement(doc41, "site", site="end2")

tree = ET.ElementTree(root)
ET.indent(tree, space="\t", level=0)
tree.write("example.xml", encoding="utf-8")
