import mujoco
model = mujoco.MjModel.from_xml_path("model/urdf/Xuanwu.urdf")
mujoco.mj_saveLastXML("model/mjcf/Xuanwu.xml", model)