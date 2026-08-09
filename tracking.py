import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import cv2
from collections import deque
import matplotlib.pyplot as plt

xml_path = 'mjmodel.xml'

simend = 10             
ORBIT_CAMERA = False      
ORBIT_DEG_PER_SEC = 6.0
TRAIL_MAXLEN = 500        
print_camera_config = 0

button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

radius = 25
x_off = 120
z_off = 120
T = 3

def init_controller(model, data):
    pass


def get_path(model, data):
    
    x = radius * np.sin(data.time / T * 2 * np.pi) + x_off
    z = radius * np.cos(data.time / T * 2 * np.pi) + z_off
    return x, z


def ik_from_task_target(xd, zd):
    pos_des = np.array([xd, zd, 60])
    x = pos_des[0]
    y = pos_des[1] + 9.744
    z = pos_des[2]
    q1 = -np.atan2(y, x)
    x1 = np.sqrt(x**2 + y**2) - 45
    z1 = z - 108.219
    D = (x1**2 + z1**2 - 103.3**2 - 109.1**2) / (2 * 103.3 * 109.1)
    if abs(D) > 1:
        return None
    q3 = np.acos(D)
    q2 = np.atan2(z1, x1) + np.atan2(109.1 * np.sin(q3), 103.3 + 109.1 * np.cos(q3))
    q4 = -q2 + q3
    return np.array([q1, q2, q3, q4])


last_q = None  


def controller(model, data):
    global last_q
    lows = model.jnt_range[:, 0]
    highs = model.jnt_range[:, 1]
    xd, zd = get_path(model, data)
    q = ik_from_task_target(xd, zd)
    if q is None:
        return
    out_of_range = np.any(q > highs) or np.any(q < lows)
    if out_of_range:
        print(f"t={data.time:.2f}s: commanded joint(s) outside range -> {np.round(q, 3)}")
    data.ctrl = q
    last_q = q


def precompute_reference_path(model, n=120):
    shadow = mj.MjData(model)
    pts = []
    for i in range(n):
        t = i / n * T
        xd = radius * np.sin(t / T * 2 * np.pi) + x_off
        zd = radius * np.cos(t / T * 2 * np.pi) + z_off
        q = ik_from_task_target(xd, zd)
        if q is None:
            continue
        shadow.qpos[:] = q
        mj.mj_kinematics(model, shadow)
        pts.append(shadow.site_xpos[0].copy())
    pts.append(pts[0])  # close the loop
    return np.array(pts)


def add_path_segments(scene, points, rgba, radius=0.0008):
    n = len(points)
    rgba = np.array(rgba, dtype=np.float32)
    for i in range(n - 1):
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                         np.zeros(9), rgba)
        mj.mjv_connector(g, mj.mjtGeom.mjGEOM_CAPSULE, radius, points[i], points[i + 1])
        scene.ngeom += 1


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global lastx, lasty
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos
    if not (button_left or button_middle or button_right):
        return
    width, height = glfw.get_window_size(window)
    mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                 glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, dx / height, dy / height, scene, cam)


def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, scene, cam)


dirname = os.path.dirname(__file__)
xml_path = os.path.join(dirname, xml_path)

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = 100.97108433734931 ; cam.elevation = -28.932530120481935 ; cam.distance =  0.565
cam.lookat =np.array([ 0.12441143828175788 , -0.004347816235532483 , 0.09157375158357003 ])
base_azimuth = cam.azimuth

fps = 60
viewport_width, viewport_height = glfw.get_framebuffer_size(window)
video = cv2.VideoWriter('jump.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps,
                         (viewport_width, viewport_height))

init_controller(model, data)
x1, y1 = get_path(model, data)
q1 = ik_from_task_target(x1, y1)
data.qpos = q1
data.qvel[:] = 0
mj.mj_forward(model, data)
mj.set_mjcb_control(controller)

reference_path = precompute_reference_path(model)
shadow_data = mj.MjData(model)  # reused each frame for the commanded-target FK
trail = deque(maxlen=TRAIL_MAXLEN)
errors = []
x_hist, y_hist = [], []

while not glfw.window_should_close(window):
    time_prev = data.time
    while data.time - time_prev < 1.0 / 60.0:
        mj.mj_step(model, data)
    if data.time >= simend:
        break

    x_hist.append(data.site_xpos[0][0])
    y_hist.append(data.site_xpos[0][1])

    if last_q is not None:
        shadow_data.qpos[:] = last_q
        mj.mj_kinematics(model, shadow_data)
        commanded_pos = shadow_data.site_xpos[0].copy()
        err = np.linalg.norm(data.site_xpos[0] - commanded_pos)
        trail.append(data.site_xpos[0].copy())
        errors.append(err)

    if ORBIT_CAMERA:
        cam.azimuth = base_azimuth + ORBIT_DEG_PER_SEC * data.time

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if print_camera_config == 1:
        print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation,
              ';', 'cam.distance = ', cam.distance)
        print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)

    add_path_segments(scene, reference_path, rgba=[0.1, 0.85, 0.2, 1.0], radius=0.0006)
    if len(trail) > 1:
        add_path_segments(scene, list(trail), rgba=[1.0, 0.1, 0.0, 1.0], radius=0.0009)

    mj.mjr_render(viewport, scene, context)

    if errors:
        overlay1 = f"t = {data.time:5.2f}s"
        overlay2 = f"tracking error = {errors[-1]*1000:5.2f} mm"
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT,
                        viewport, overlay1, overlay2, context)

    rgb = np.empty((viewport.height, viewport.width, 3), dtype=np.uint8)
    depth = np.empty((viewport.height, viewport.width), dtype=np.float32)
    mj.mjr_readPixels(rgb, depth, viewport, context)
    rgb = np.flipud(rgb)
    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    video.write(frame)

    glfw.swap_buffers(window)
    glfw.poll_events()

video.release()
glfw.terminate()

# static asset for the README: reference vs actual, plus error over time
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
axes[0].plot(reference_path[:, 0], reference_path[:, 1], '--', color='green', label='reference')
axes[0].plot(x_hist, y_hist, color='red', linewidth=1, label='actual')
axes[0].set_aspect('equal')
axes[0].set_xlabel('x (m)')
axes[0].set_ylabel('y (m)')
axes[0].set_title('Pen-tip path: commanded vs actual')
axes[0].legend()

axes[1].plot(np.array(errors) * 1000)
axes[1].set_xlabel('frame')
axes[1].set_ylabel('tracking error (mm)')
axes[1].set_title('Tracking error over time')

plt.tight_layout()
plt.savefig('tracking_summary.png', dpi=150)
plt.show()