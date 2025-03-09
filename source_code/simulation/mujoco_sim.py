import mujoco
import mujoco.viewer as viewer
import numpy as np

m = mujoco.MjModel.from_xml_path("model/mjcf/Xuanwu.xml")
d = mujoco.MjData(m)

init_joint_pos = np.array([0.0, 0.0, 0.35, -0.5, 0.35, 0.0, 0.0, 0.35, -0.5, 0.35])
init_base_pos = np.array([0, 0, 1])
init_base_eular_zyx = np.array([0.0, 0.0, 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])

viewer.launch(m,d)
