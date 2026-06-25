import numpy as np
import argparse
import os, shutil
import gmsh
import math
import trimesh
from pathlib import Path
import subprocess
import random

import common

parser = argparse.ArgumentParser(description='Launch CAO simulation with ROS node.')
parser.add_argument('--index', type=int, default=-1, help='The index in the loaded CAO files')
parser.add_argument("--additional-translation", type=float, nargs=3, default=[0,0,0], help='Additional translation for the CT -> VB transformation.')

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

def nominalRegistration(tumor_msh, trachea_stl, fixed_faces_txt, additional_translation):

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
    trachea_axis = np.copy(T[:3, long_idx])

    reference_point = tumor.vertices.mean(axis=0)
    obb_center = T[:3,3]
    if np.dot(trachea_axis, reference_point - obb_center) < 0:
        trachea_axis *= -1

    # get normal from one of the fixed faces of the tumor - this is the x-axis
    ff_normal = tumor.face_normals[ff_ind]
    ff_centroid = 1/3 * (tumor.vertices[tumor.faces[ff_ind][0]] + tumor.vertices[tumor.faces[ff_ind][1]] + tumor.vertices[tumor.faces[ff_ind][2]])
    # ff_centroid = np.mean(tumor.vertices[tumor.faces[ff_ind]], axis=0)

    # y-axis = z cross x
    y_axis = np.cross(trachea_axis, ff_normal)

    # x-axis = y cross z
    x_axis = np.cross(y_axis, trachea_axis)

    # normalize axes
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = trachea_axis / np.linalg.norm(trachea_axis)

    # make sure tumor is on the right
    # if np.dot(x_axis, ff_centroid - obb_center) < 0:
        # flip frame to make tumor on +x
        

    # assemble rotation matrix
    R = np.zeros((3,3))
    R[:,0] = x_axis
    R[:,1] = y_axis
    R[:,2] = z_axis

    

    # get Euler angles
    from scipy.spatial.transform import Rotation as Rot
    r = Rot.from_matrix(R)
    eul_XYZ = np.rad2deg(r.as_euler('xyz'))

    # heuristic for desired position: project tumor centroid onto trachea axis, then move VB position 20 mm back from this point
    tumor_centroid = tumor.vertices.mean(axis=0)
    proj_scalar = np.dot(tumor_centroid - T[:3,3], z_axis)
    tumor_trachea_center = T[:3,3] + proj_scalar * z_axis
    offset = np.add([0,-2,30], additional_translation)
    t = -R @ tumor_trachea_center + offset
    

    tumor_in_frame = R @ ff_centroid + t
    print(tumor_in_frame)
    if tumor_in_frame[0] > 0:
        x_axis *= -1
        y_axis *= -1

        R = np.zeros((3,3))
        R[:,0] = x_axis
        R[:,1] = y_axis
        R[:,2] = z_axis

        r = Rot.from_matrix(R)
        eul_XYZ = np.rad2deg(r.as_euler('xyz'))

        proj_scalar = np.dot(tumor_centroid - T[:3,3], z_axis)
        tumor_trachea_center = T[:3,3] + proj_scalar * z_axis
        t = -R @ tumor_trachea_center + offset

    print(R)
    print(t)

    tumor_in_frame = R @ ff_centroid + t
    print(tumor_in_frame[0])

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
    args = parser.parse_args()

    tumor_mshs, trachea_stls = indexCAOs()

    gmsh.initialize()
    ind = args.index
    if (ind == -1):
        ind = random.randint(0, len(tumor_mshs)-1)

    # get fixed faces file from tumor msh filename
    fixed_faces_txt = os.path.splitext(tumor_mshs[ind])[0] + "_fixed_faces.txt"
    eul_XYZ, trans = nominalRegistration(tumor_mshs[ind], trachea_stls[ind], fixed_faces_txt, args.additional_translation)

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