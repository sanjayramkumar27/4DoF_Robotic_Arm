import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(r"C:\Users\santh\Desktop\Robottic_Arm\vs\Robotic_Arm\arm_urdf\urdf\arm_urdf.urdf")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        #mujoco.mj_step(model, data)
        viewer.sync()