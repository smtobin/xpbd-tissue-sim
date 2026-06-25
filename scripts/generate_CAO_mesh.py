import numpy as np
import pymeshlab
import argparse
import os, shutil
import gmsh
import math
import trimesh
import scipy
import collections

import common

parser = argparse.ArgumentParser(description='Generate necessary files for CAO simulation')
parser.add_argument('tumor_stl', help='Tumor STL file (from CT)')
parser.add_argument('trachea_stl', help='Trachea STL file (from CT)')
parser.add_argument('--tumor-vertices', type=int, default=800, help='Number of target vertices for the tumor mesh.')
parser.add_argument('--trachea-vertices', type=int, default=1500, help='Number of target vertices for the trachea mesh.')
parser.add_argument('-o1', '--tumor-output-msh', default='tumor.msh', help='Output .msh file for the tumor (optional).')
parser.add_argument('-o2', '--trachea-output-stl', default='trachea.stl', help='Output .stl file for the trachea (optional).')

def main():
    # create a tmp directory for saving temp files
    tmp_dir = ".tmp"
    if not os.path.isdir(tmp_dir):
        os.mkdir(tmp_dir)

    # process args
    args = parser.parse_args()

    # process/decimate tumor and trachea
    print(f"=== Processing tumor mesh ===")
    processed_tumor_stl = tmp_dir + "/tumor.stl"
    common.decimateToTargetNumberOfVertices(args.tumor_stl, args.tumor_vertices, processed_tumor_stl)

    print(f"=== Processing trachea mesh ===")
    processed_trachea_stl = args.trachea_output_stl
    common.decimateToTargetNumberOfVertices(args.trachea_stl, args.trachea_vertices, processed_trachea_stl)

    # convert tumor mesh to .msh with gmsh
    gmsh.initialize()
    tumor_msh_file = args.tumor_output_msh
    common.generateMSHfromSTL(processed_tumor_stl, tumor_msh_file)

    # get surface of .msh (triangle may change when GMSH converts from surface to volume mesh)
    tumor_msh_surface_file = tmp_dir + "/tumor_msh_surface.obj"
    common.getSurfaceOBJFromMSH(tumor_msh_file, tumor_msh_surface_file)

    # load both decimated meshes
    ms = pymeshlab.MeshSet()

    ms.load_new_mesh(tumor_msh_surface_file)
    meshA_ml = ms.current_mesh()

    ms.load_new_mesh(processed_trachea_stl)
    meshB_ml = ms.current_mesh()

    # Extract vertices/faces
    VA = meshA_ml.vertex_matrix()
    FA = meshA_ml.face_matrix()

    VB = meshB_ml.vertex_matrix()
    FB = meshB_ml.face_matrix()

    # Create trimesh objects
    meshA = trimesh.Trimesh(vertices=VA, faces=FA, process=False)
    meshB = trimesh.Trimesh(vertices=VB, faces=FB, process=False)

    # === Find tumor-trachea boundary ===

    # Face centroids of tumor
    centroids = meshA.triangles_center

    # Signed distance to trachea
    dist = -trimesh.proximity.signed_distance(meshB, centroids)

    # use 0.5 mm threshold distance
    threshold = 0.5
    close_faces = np.where(dist < threshold)[0]

    print(f"{len(close_faces)} close faces")

    # === Erode the set of faces at the tumor-trachea boundary ===
    selected = set(close_faces)
    eroded = common.erodeSelectionByArea(selected, VA, FA, 0.95)

    # generate the fixed faces file from the eroded selection
    fixed_faces_filename = os.path.splitext(args.tumor_output_msh)[0] + "_fixed_faces.txt"
    with open(fixed_faces_filename, "w") as fixed_faces_file:
        for f_ind in eroded:
            face = FA[f_ind]
            fixed_faces_file.write(f"3 {face[0]} {face[1]} {face[2]} {f_ind}\n")

    # === Compute nominal registration ===
    # get axis down the trachea - this is the z-axis
    obb = meshB.bounding_box_oriented
    T = obb.primitive.transform
    extents = obb.primitive.extents
    long_idx = np.argmax(extents)
    trachea_axis = T[:3, long_idx]
    

    # get normal from one of the fixed faces of the tumor - this is the x-axis
    ff_normal = meshA.face_normals[list(eroded)[0]]

    # y-axis = z cross x
    y_axis = np.cross(trachea_axis, ff_normal)

    # x-axis = y cross z
    x_axis = np.cross(y_axis, trachea_axis)

    # normalize axes
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = trachea_axis / np.linalg.norm(trachea_axis)

    # assemble rotation matrix
    R = np.zeros((3,3))
    R[:,0] = x_axis
    R[:,1] = y_axis
    R[:,2] = z_axis

    print(R)

    from scipy.spatial.transform import Rotation as Rot
    r = Rot.from_matrix(R)
    print(np.rad2deg(r.as_euler('xyz')))

    # get Euler angles
    eul_XYZ = common.rotation_matrix_to_euler_xyz(R)
    print(np.rad2deg(eul_XYZ))

    # we want tumor centroid to be 20 mm in the z direction
    tumor_centroid = VA.mean(axis=0)
    p_des = [0,0,0]
    t = p_des - R @ tumor_centroid
    print(t)

    patch = meshA.submesh([list(eroded)], append=True)

    meshA_vis = meshA.copy()
    meshA_vis.visual.face_colors = [180, 180, 180, 100]
    meshB_vis = meshB.copy()
    meshB_vis.visual.face_colors = [200, 200, 200, 100]

    patch.visual.face_colors = [255, 0, 0, 255]

    origin = VB.mean(axis=0)
    L = 50

    x_line = trimesh.load_path(
        np.vstack([origin, origin + L * x_axis])
    )
    y_line = trimesh.load_path(
        np.vstack([origin, origin + L * y_axis])
    )
    z_line = trimesh.load_path(
        np.vstack([origin, origin + L * z_axis])
    )

    # color them
    x_line.colors = [[255,0,0,255]]
    y_line.colors = [[0,255,0,255]]
    z_line.colors = [[0,0,255,255]]

    trimesh.Scene([
        meshA_vis,
        patch,
        meshB_vis,
        x_line,
        y_line,
        z_line
    ]).show(smooth=False)

    shutil.rmtree(tmp_dir)

if __name__ == '__main__':
    main()