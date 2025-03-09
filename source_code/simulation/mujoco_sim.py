import os
os.environ["MUJOCO_GL"] = "egl"
import mujoco
import mujoco.viewer as viewer
import numpy as np

m = mujoco.MjModel.from_xml_path("model/mjcf/Xuanwu.xml")
d = mujoco.MjData(m)

viewer.launch(m,d)
