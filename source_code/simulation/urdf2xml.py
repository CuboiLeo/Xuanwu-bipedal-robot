from urdf2mjcf import run

run(
    urdf_path='model/urdf/Xuanwu.urdf',
    mjcf_path='model/mjcf/Xuanwu.mjcf',
    copy_meshes=True,
)