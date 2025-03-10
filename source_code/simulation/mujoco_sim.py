import mujoco
import mujoco.viewer as viewer
import numpy as np
import time

m = mujoco.MjModel.from_xml_path("model/mjcf/Xuanwu.xml")#("humanoid_legged.xml")
d = mujoco.MjData(m)

init_joint_pos = np.array([0.0, 0.0, 0.35, -0.5, 0.35, 0.0, 0.0, 0.35, -0.5, 0.35])
init_base_pos = np.array([0, 0, 1])
init_base_quat = np.array([1.0, 0.0, 0.0, 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])

d.qpos[:3] = init_base_pos
d.qpos[3:7] = init_base_quat
d.qpos[-10:] = init_joint_pos
d.qvel[:3] = np.array([0.0, 0.0, 0.0])
d.qvel[-10:] = np.zeros(10)

mujoco.mj_step(m, d)
viewer.launch(m, d)