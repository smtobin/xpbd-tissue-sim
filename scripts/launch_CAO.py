import numpy as np
import argparse
import os, shutil
import gmsh
import math
import trimesh
from pathlib import Path
import subprocess

import common

def indexCAOs():
    # search through resource/lfs/CAO for tumor and trachea mesh files
    trachea_stls = []
    tumor_mshs = []

    root = "../resource/lfs/CAO"

    for dirpath, dirnames, filenames in os.walk(root):
        for filename in filenames:
            full_path = Path(os.path.join(dirpath, filename))
            
            # find tumor .msh files
            if full_path.suffix.lower() == ".msh":

                # look for trachea .stl file
                prefix = filename.split("tumor")[0].rstrip("_- ")
                for other_file in filenames:
                    if prefix in other_file and "trachea" in other_file:
                        tumor_mshs.append(str(full_path))
                        trachea_stls.append(os.path.join(dirpath, other_file))
                        break
            
    return (tumor_mshs, trachea_stls)

def nominalRegistration(tumor_msh, trachea_stl, fixed_faces_txt):

    tmp_dir = ".tmp"
    if not os.path.isdir(tmp_dir):
        os.mkdir(tmp_dir)

    # get surface of tumor .msh file
    tumor_msh_surface_file = tmp_dir + "/tumor_msh_surface.obj"
    common.getSurfaceOBJFromMSH(tumor_msh, tumor_msh_surface_file)

    # load tumor and trachea mesh files
    tumor = trimesh.load(tumor_msh_surface_file)
    trachea = trimesh.load(trachea_stl)

    # get first fixed face in fixed faces txt file
    ff_ind = None
    with open(fixed_faces_txt, 'r') as fixed_faces_file:
        for line in fixed_faces_file:
            ff_ind = int(line.split(' ')[4])
            break

    # === Compute nominal registration ===
    # get axis down the trachea - this is the z-axis
    obb = trachea.bounding_box_oriented
    T = obb.primitive.transform
    extents = obb.primitive.extents
    long_idx = np.argmax(extents)
    trachea_axis = T[:3, long_idx]

    # get normal from one of the fixed faces of the tumor - this is the x-axis
    ff_normal = tumor.face_normals[ff_ind]

    # y-axis = z cross x
    y_axis = np.cross(trachea_axis, ff_normal)

    # x-axis = y cross z
    x_axis = np.cross(y_axis, trachea_axis)

    # normalize axes
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = trachea_axis / np.linalg.norm(trachea_axis)

    # make sure tumor is on the right
    c_diff = T[:3,3] - tumor.vertices.mean(axis=0)
    if np.dot(x_axis, c_diff) < 0:
        # flip frame to make tumor on +x
        x_axis *= -1
        y_axis *= -1

    # assemble rotation matrix
    R = np.zeros((3,3))
    R[:,0] = x_axis
    R[:,1] = y_axis
    R[:,2] = z_axis

    print(R)

    # get Euler angles
    from scipy.spatial.transform import Rotation as Rot
    r = Rot.from_matrix(R)
    eul_XYZ = np.rad2deg(r.as_euler('xyz'))

    # heuristic for desired position: project tumor centroid onto trachea axis, then move VB position 20 mm back from this point
    tumor_centroid = tumor.vertices.mean(axis=0)
    proj_scalar = np.dot(tumor_centroid - T[:3,3], z_axis)
    tumor_trachea_center = T[:3,3] + proj_scalar * z_axis
    t = -R @ tumor_trachea_center + [0,0,25]
    print(t)

    # uncomment for visualization
    # meshA_vis = tumor.copy()
    # meshA_vis.visual.face_colors = [180, 180, 180, 100]
    # meshB_vis = trachea.copy()
    # meshB_vis.visual.face_colors = [200, 200, 200, 100]

    # origin = trachea.vertices.mean(axis=0)
    # L = 50

    # x_line = trimesh.load_path(
    #     np.vstack([origin, origin + L * x_axis])
    # )
    # y_line = trimesh.load_path(
    #     np.vstack([origin, origin + L * y_axis])
    # )
    # z_line = trimesh.load_path(
    #     np.vstack([origin, origin + L * z_axis])
    # )

    # # color them
    # x_line.colors = [[255,0,0,255]]
    # y_line.colors = [[0,255,0,255]]
    # z_line.colors = [[0,0,255,255]]

    # trimesh.Scene([
    #     meshA_vis,
    #     meshB_vis,
    #     x_line,
    #     y_line,
    #     z_line
    # ]).show(smooth=False)

    shutil.rmtree(tmp_dir)

    return eul_XYZ, t

def main():
    tumor_mshs, trachea_stls = indexCAOs()
    print(tumor_mshs)
    print(trachea_stls)

    gmsh.initialize()

    ind = 5

    # get fixed faces file from tumor msh filename
    fixed_faces_txt = os.path.splitext(tumor_mshs[ind])[0] + "_fixed_faces.txt"
    eul_XYZ, trans = nominalRegistration(tumor_mshs[ind], trachea_stls[ind], fixed_faces_txt)

    # convert translation to meters
    trans /= 1000

    cmd = [
        "ros2",
        "launch",
        "launch/CAO_sim_bridge.launch.py",
        f"tumor_mesh_filename:={tumor_mshs[ind]}",
        f"trachea_mesh_filename:={trachea_stls[ind]}",
        f"CT_to_VB_translation:=[{trans[0]:.5f},{trans[1]:.5f},{trans[2]:.5f}]",
        f"CT_to_VB_rotation:=[{eul_XYZ[0]:.3f},{eul_XYZ[1]:.3f},{eul_XYZ[2]:.3f}]"
    ]

    subprocess.run(cmd)

if __name__ == '__main__':
    main()