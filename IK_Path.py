import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt

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

def get_path(model, data):
    T=5
    x = 20*np.sin(data.time/T*2*np.pi) + 110
    z = 20*np.cos(data.time/T*2*np.pi) + 110

    return x,z

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    lows = model.jnt_range[:, 0]
    highs = model.jnt_range[:, 1]
    xd, yd = get_path(model, data)
    pos_des = [xd, yd, 60]
    pos_des = np.array(pos_des)
    x = pos_des[0]
    y = pos_des[1] + 9.744
    z = pos_des[2]
    q1 = -np.atan2(y, x)
    x1 = np.sqrt(x**2+y**2) - 45
    z1 = z -  108.219
    D = (x1**2+z1**2 - 103.3**2 - 109.1**2)/(2*103.3*109.1)
    q3 = np.acos(D) 
    q2 = np.atan2(z1,x1) + np.atan2((109.1*np.sin(q3)),(103.3+109.1*np.cos(q3))) 
    q4 = -q2+q3
    q = [q1,q2,q3,q4]
    if not np.greater(highs,q).all() and not np.less(lows, q).all():
        print("Position outside workspace")
    data.ctrl = q



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

import cv2

fps = 60

viewport_width, viewport_height = glfw.get_framebuffer_size(window)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter(
    'jump.mp4',
    fourcc,
    fps,
    (viewport_width, viewport_height)
)

#initialize the controller
init_controller(model,data)
data.qpos = np.deg2rad([0, 0, 0, 0])
data.qvel[:] = 0
mj.mj_forward(model, data)

#set the controller
mj.set_mjcb_control(controller)
x = []
y=[]
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)  
        x.append(data.site_xpos[0][0])
        y.append(data.site_xpos[0][1])
    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    rgb = np.empty((viewport.height, viewport.width, 3), dtype=np.uint8)
    depth = np.empty((viewport.height, viewport.width), dtype=np.float32)

    mj.mjr_readPixels(rgb, depth, viewport, context)

    # OpenGL image is upside down
    rgb = np.flipud(rgb)

    # OpenCV expects BGR
    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    video.write(frame)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()
video.release()
glfw.terminate()


plt.plot(x,y)
plt.show()