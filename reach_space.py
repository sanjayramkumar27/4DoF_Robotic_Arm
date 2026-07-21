import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path = 'mjmodel.xml' #xml file (assumes this is in the same folder as this file)
simend = 50 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    '''pos_des = [100, 0.0, 100.0]
    pos_des = np.array(pos_des)
    x = pos_des[0]
    y = pos_des[1] - 9.744/2
    z = pos_des[2]
    q1 = np.atan2(y, x)
    x1 = np.sqrt(x**2+y**2) + 37
    z1 = z -  108.219
    D = (x1**2+z1**2 - 103.3**2 - 109.1**2)/(2*103.3*109.1)
    q3 = np.acos(D) 
    q2 = np.atan2(z1,x1) + np.atan2((109.1*np.sin(q3)),(103.3+109.1*np.cos(q3))) 
    q4 = -q2+q3
    q = [q1,q2,q3,q4]
    data.ctrl = q
    print(np.rad2deg(q))
    print(pos_des-data.site_xpos[0]*1000)'''
    pass



def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])

cam.azimuth = 88.19999999999995 ; cam.elevation = -15.799999999999974 ; cam.distance =  0.5653395162666803
cam.lookat =np.array([ -0.007058707059627407 , -0.0009162690436801375 , 0.0568240777320992 ])

#initialize the controller
init_controller(model,data)
data.qpos = np.deg2rad([0, 0, 0, 0])
data.qvel[:] = 0
mj.mj_forward(model, data)
print(data.site_xpos[0])
#set the controller
mj.set_mjcb_control(controller)


rng = np.random.default_rng(10)
lows = model.jnt_range[:, 0]
highs = model.jnt_range[:, 1]

n_samples = 10000
pts = np.zeros((n_samples, 3))
for i in range(n_samples):
    q = rng.uniform(lows, highs)
    data.qpos[:model.nq] = q
    mj.mj_forward(model, data)
    pts[i] = data.site_xpos[0]


print("Reachable pen-tip bounding box:")
for axis, name in zip(range(3), "xyz"):
    print(f"  {name}: [{pts[:, axis].min():.3f}, {pts[:, axis].max():.3f}]")
centroid = pts.mean(axis=0)
print(f"  centroid: {centroid}")


import matplotlib.pyplot as plt

fig = plt.figure(figsize=(12, 4))

# 3D view
ax1 = fig.add_subplot(131, projection="3d")
ax1.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=2, alpha=0.3)
ax1.set_xlabel("x"); ax1.set_ylabel("y"); ax1.set_zlabel("z")
ax1.set_title("3D reachable cloud")

# top-down (x-y) — useful for picking where on a table to draw
ax2 = fig.add_subplot(132)
ax2.scatter(pts[:, 0], pts[:, 1], s=2, alpha=0.3)
ax2.set_xlabel("x"); ax2.set_ylabel("y")
ax2.set_aspect("equal")
ax2.set_title("top-down (x-y)")

# side view (x-z) — useful for picking table height
ax3 = fig.add_subplot(133)
ax3.scatter(pts[:, 0], pts[:, 2], s=2, alpha=0.3)
ax3.set_xlabel("x"); ax3.set_ylabel("z")
ax3.set_aspect("equal")
ax3.set_title("side view (x-z)")

fig.tight_layout()
fig.savefig("workspace_cloud.png", dpi=150)



