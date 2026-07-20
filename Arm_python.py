import serial
import numpy as np
import pyvista as pv
import threading
import time

# ---------- SERIAL ----------
ser = serial.Serial('COM3', 115200)
ser.timeout = 0.1

# ---------- HOMOGENEOUS TRANSFORMS ----------
def h1(t1):
    return np.array([
        [np.cos(t1), -np.sin(t1), 0, 0],
        [np.sin(t1),  np.cos(t1), 0, 0],
        [0, 0, 1, 1278/25],
        [0, 0, 0, 1]
    ])
def h2(t2):
    return np.array([
        [ np.cos(t2), 0, np.sin(t2), -201/20],
        [ 0, 1, 0, 129/50],
        [-np.sin(t2), 0, np.cos(t2), 95/2],
        [0, 0, 0, 1]
    ])
def h3(t3):
    return np.array([
        [ np.cos(t3), 0, np.sin(t3), 1033/10],
        [ 0, 1, 0, 423/25],
        [-np.sin(t3), 0, np.cos(t3), 5/2],
        [0, 0, 0, 1]
    ])
def h4(t4):
    return np.array([
        [ np.cos(t4), 0, np.sin(t4), 2133/20],
        [ 0, 1, 0, -71/5],
        [-np.sin(t4), 0, np.cos(t4), -42/25],
        [0, 0, 0, 1]
    ])
def h5():
    return np.array([
        [1, 0, 0, 42.55],
        [0, 1, 0, 9.68],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

# ---------- FORWARD KINEMATICS ----------
def forward_kinematics(t1, t2, t3, t4):
    T0 = np.eye(4)
    T1 = T0 @ h1(t1)
    T2 = T1 @ h2(t2)
    T3 = T2 @ h3(t3)
    T4 = T3 @ h4(t4)
    T5 = T4 @ h5()
    positions = [
        T0[:3, 3], T1[:3, 3], T2[:3, 3],
        T3[:3, 3], T4[:3, 3], T5[:3, 3]
    ]
    return positions, T5

# ---------- SHARED STATE ----------
latest_positions = [None]
lock = threading.Lock()

# ---------- SERIAL THREAD ----------
def serial_thread():
    while True:
        try:
            line = ser.readline().decode().strip()
            if not line:
                continue
            vals = [float(x) for x in line.split(',')]
            if len(vals) != 4:
                continue
            t1, t2, t3, t4 = [np.deg2rad(v) for v in vals]
            positions, T = forward_kinematics(t1, -t2, -t3, t4)
            with lock:
                latest_positions[0] = positions
            print("EE Position:", T[:3, 3])
        except Exception as e:
            print("Error:", e)

# ---------- PYVISTA SETUP ----------
pv.global_theme.background = '#0a0a1a'

NUM_LINKS  = 5
NUM_JOINTS = 6
colors      = ['#0088ff']*NUM_LINKS
joint_colors = ['#ffffff'] * NUM_JOINTS
#joint_colors[-1] = '#ff3333'
joint_sizes  = [10, 7, 7, 7, 7, 7]

plotter = pv.Plotter(window_size=[900, 700])
plotter.add_axes()
plotter.camera_position = [(600, 400, 400), (50, 0, 100), (0, 0, 1)]

# ---------- PRE-CREATE MESHES (never removed, just overwritten) ----------
link_meshes  = []
joint_meshes = []

for i in range(NUM_LINKS):
    cyl = pv.Cylinder(center=[0, 0, i*10], direction=[0, 0, 1],
                      radius=4, height=10, resolution=20)
    plotter.add_mesh(cyl, color=colors[i], smooth_shading=True)
    link_meshes.append(cyl)

for i in range(NUM_JOINTS):
    sph = pv.Sphere(radius=joint_sizes[i], center=[0, 0, i*10])
    plotter.add_mesh(sph, color=joint_colors[i], smooth_shading=True)
    joint_meshes.append(sph)

# ---------- UPDATE GEOMETRY IN-PLACE (no flash!) ----------
def update_scene():
    with lock:
        positions = latest_positions[0]
    if positions is None:
        return

    for i in range(NUM_LINKS):
        p0 = np.array(positions[i],     dtype=float)
        p1 = np.array(positions[i + 1], dtype=float)
        direction = p1 - p0
        length = np.linalg.norm(direction)
        if length < 0.01:
            direction = np.array([0, 0, 1], dtype=float)
            length = 0.01

        # Build new geometry and copy points/cells into existing mesh
        new_cyl = pv.Cylinder(center=(p0 + p1) / 2, direction=direction,
                              radius=4, height=length, resolution=20)
        link_meshes[i].copy_from(new_cyl)

    for i, p in enumerate(positions):
        new_sph = pv.Sphere(radius=joint_sizes[i], center=np.array(p, dtype=float))
        joint_meshes[i].copy_from(new_sph)

# ---------- START ----------
t = threading.Thread(target=serial_thread, daemon=True)
t.start()

plotter.show(auto_close=False, interactive_update=True, title="Robot Arm Visualizer")

while True:
    update_scene()
    plotter.update()
    time.sleep(0.05)