import numpy as np
import argparse
import os, shutil
import pymeshlab
import collections
import gmsh
import math
import trimesh
from pathlib import Path
import subprocess
import random
from stl import mesh

import common

parser = argparse.ArgumentParser(description='Process STL mesh files')
parser.add_argument('--prostate-stl', help='Prostate surface STL file (From CT)')
parser.add_argument('--lesion-stl', help='Lesion surface STL file (from CT)')
parser.add_argument('--prostate-msh', help='Prostate .msh file with lesion. Assumes <filename>_element_classes.txt and <filename>_fixed_faces.txt exist.')
parser.add_argument('--lesion-vertices', type=int, default=100, help='Numer of target vertices for the lesion mesh')
parser.add_argument('--prostate-vertices', type=int, default=1000, help='Number of target vertices for the prostate mesh')
parser.add_argument("--lesion-translation", type=float, nargs=3, default=[0,0,0], help='Additional translation [mm] of the lesion input mesh.')
parser.add_argument("--lesion-rotation", type=float, nargs=3, default=[0,0,0], help='Additional rotation (XYZ Euler angles, deg) of the lesion input mesh.')
parser.add_argument("--lesion-scaling", type=float, nargs=3, default=[1,1,1], help='Scaling of the lesion mesh along X,Y,Z axes.')
parser.add_argument('-o', '--output-msh', default='prostate_with_lesion.msh', help='Output .msh file (optional)')
parser.add_argument('--config-filename', default='../config/demos/virtuoso_prostate/focal_lesion.yaml', help='.yaml config file to use for the sim. Passed onto the ROS launch.')

def face_key(n1, n2, n3):
    # stable identity: node IDs (NOT coordinates)
    return tuple(sorted((n1, n2, n3)))

# ----------------------------
# Direction sampling (Fibonacci sphere)
# ----------------------------
def fibonacci_sphere(samples=300):
    points = []
    offset = 2.0 / samples
    increment = np.pi * (3.0 - np.sqrt(5.0))

    for i in range(samples):
        y = ((i * offset) - 1) + (offset / 2)
        r = np.sqrt(max(0.0, 1 - y * y))
        phi = i * increment

        x = np.cos(phi) * r
        z = np.sin(phi) * r

        points.append([x, y, z])

    return np.array(points)


# ----------------------------
# Build orthonormal basis from direction
# ----------------------------
def make_basis(d):
    d = d / np.linalg.norm(d)

    # pick helper vector not parallel to d
    up = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(d, up)) > 0.9:
        up = np.array([1.0, 0.0, 0.0])

    x = np.cross(up, d)
    x /= np.linalg.norm(x)

    y = np.cross(d, x)

    return x, y, d


# ----------------------------
# Project vertices into 2D plane orthogonal to d
# ----------------------------
def project_vertices(vertices, basis):
    x, y, _ = basis
    return np.column_stack([
        vertices @ x,
        vertices @ y
    ])


# ----------------------------
# Compute convex hull area in 2D
# ----------------------------
def convex_hull_area(points_2d):
    if len(points_2d) < 3:
        return 0.0

    # Monotonic chain convex hull (no scipy dependency)
    pts = np.array(sorted(points_2d.tolist()))
    
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))

    hull = lower[:-1] + upper[:-1]

    def polygon_area(poly):
        area = 0.0
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            area += x1 * y2 - x2 * y1
        return abs(area) * 0.5

    return polygon_area(hull)


# ----------------------------
# Score projection openness
# ----------------------------
def projection_score(mesh, direction):
    direction = direction / np.linalg.norm(direction)

    # ---- 1. Surface collapse score (important) ----
    normals = mesh.face_normals
    areas = mesh.area_faces

    # how much each face "faces" the view direction
    collapse = np.sum(areas * np.abs(normals @ direction))

    # lower collapse = more edge-on = more likely tunnel axis
    collapse_score = 1.0 / (collapse + 1e-9)

    # ---- 2. Vertex spread in projection ----
    basis = make_basis(direction)
    proj = project_vertices(mesh.vertices, basis)

    hull_area = convex_hull_area(proj)

    # normalize spread (avoid scale bias)
    var = np.var(proj, axis=0)
    spread = np.sqrt(var[0] * var[1]) + 1e-9

    openness = hull_area / spread

    # ---- combined score ----
    return collapse_score * openness


# ----------------------------
# Main function: find tunnel axis
# ----------------------------
def find_tunnel_axis(mesh, samples=400):
    directions = fibonacci_sphere(samples)

    best_dir = None
    best_score = -np.inf

    for d in directions:
        score = projection_score(mesh, d)

        if score > best_score:
            best_score = score
            best_dir = d

    return best_dir, best_score

def main():
    args = parser.parse_args()

    print(f"Creating unified STL mesh with outer surface from {args.prostate_stl} and inner surface from {args.lesion_stl}...")

    # create a tmp directory for saving temp files
    tmp_dir = ".tmp"
    if not os.path.isdir(tmp_dir):
        os.mkdir(tmp_dir)

    # process/decimate prostate and lesion
    print(f"=== Processing lesion mesh ===")
    processed_lesion_stl = tmp_dir + "/lesion.stl"
    common.decimateToTargetNumberOfVertices(args.lesion_stl, args.lesion_vertices, processed_lesion_stl)

    print(f"=== Processing prostate mesh ===")
    processed_prostate_stl = tmp_dir + "/prostate.stl"
    common.decimateToTargetNumberOfVertices(args.prostate_stl, args.prostate_vertices, processed_prostate_stl)

    # === Apply transform to lesion ===
    # get lesion centroid
    lesion = mesh.Mesh.from_file(processed_lesion_stl)
    v = lesion.vectors.reshape(-1, 3)

    c = v.mean(axis=0)
    v0 = v - c

    rx, ry, rz = np.deg2rad(args.lesion_rotation)

    Rx = np.array([[1,0,0],
                [0,np.cos(rx),-np.sin(rx)],
                [0,np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry),0,np.sin(ry)],
                [0,1,0],
                [-np.sin(ry),0,np.cos(ry)]])

    Rz = np.array([[np.cos(rz),-np.sin(rz),0],
                [np.sin(rz), np.cos(rz),0],
                [0,0,1]])

    R = Rx @ Ry @ Rz

    S = np.diag(args.lesion_scaling)

    v_final = (R @ (S @ v0.T)).T + c + args.lesion_translation

    lesion.vectors = v_final.reshape(-1, 3, 3)
    transformed_lesion_stl = tmp_dir + "/lesion_transformed.stl"
    lesion.save(transformed_lesion_stl)

    # load mesh with MeshLab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(processed_prostate_stl)
    ms.load_new_mesh(transformed_lesion_stl)

    # use boolean difference to combine the meshes
    ms.generate_boolean_difference(
        first_mesh=0,
        second_mesh=1
    )

    result_id = ms.number_meshes() - 1

    hollow_stl = tmp_dir + "/hollow.stl"
    ms.set_current_mesh(result_id)
    ms.save_current_mesh(hollow_stl)

    gmsh.initialize()
    hollow_msh = tmp_dir + "/hollow.msh"
    lesion_msh = tmp_dir + "/lesion.msh"
    common.generateMSHfromSTL(hollow_stl, hollow_msh)
    common.generateMSHfromSTL(transformed_lesion_stl, lesion_msh)

    gmsh.clear()
    gmsh.model.add("merged")

    gmsh.merge(hollow_msh)
    gmsh.merge(lesion_msh)

    gmsh.model.mesh.removeDuplicateNodes()
    gmsh.model.mesh.removeDuplicateElements()

    entities = gmsh.model.getEntities(2)

    for dim, ent_tag in entities:
        elem_types, elem_tags, _ = gmsh.model.mesh.getElements(dim, ent_tag)

        total = sum(len(t) for t in elem_tags)
        # print(f"Surface {ent_tag}: {total} elements")

    # -------------------------------------------------------
    # Get all 2D elements (triangles)
    # -------------------------------------------------------
    elem_types, elem_tags, elem_nodes = gmsh.model.mesh.getElements(2)

    face_map = collections.defaultdict(list)

    # -------------------------------------------------------
    # Build face -> element map
    # -------------------------------------------------------
    for etype, tags, nodes in zip(elem_types, elem_tags, elem_nodes):

        # 2 = 3-node triangle in Gmsh
        if etype != 2:
            continue

        nodes = np.array(nodes).reshape(-1, 3)

        for tag, tri in zip(tags, nodes):
            key = face_key(*tri)
            face_map[key].append(tag)

    for key, tags in face_map.items():
        if len(tags) == 2:
            break


    element_to_surface = {}

    for dim, surf_tag in gmsh.model.getEntities(2):
        elem_types, elem_tags, _ = gmsh.model.mesh.getElements(dim, surf_tag)

        for tags in elem_tags:
            for e in tags:
                element_to_surface[int(e)] = surf_tag


    

    # -------------------------------------------------------
    # Identify interface faces (shared by 2 elements)
    # -------------------------------------------------------
    interface_elements = set()

    for key, tags in face_map.items():
        if len(tags) == 2:
            interface_elements.update(tags)

    interface_elements = list(interface_elements)

    print(f"\n === Found {len(interface_elements)} interface triangles ===")

    remove_by_surface = collections.defaultdict(list)

    for e in interface_elements:
        surf = element_to_surface[int(e)]
        remove_by_surface[surf].append(int(e))

    for surf, elems in remove_by_surface.items():
        print(f"  Removing {len(elems)} elements from surface {surf}")
        gmsh.model.mesh.removeElements(2, surf, elems)

    print("\n")

    gmsh.write(args.output_msh)

    gmsh.clear()

    gmsh.open(args.output_msh)
    # get tetra elements (type 4)
    elem_tags, elem_nodes = gmsh.model.mesh.getElementsByType(4)

    # get all node coordinates
    node_ids, node_coords, _ = gmsh.model.mesh.getNodes()
    coords = node_coords.reshape(-1, 3)

    # map node ID → coordinate
    node_map = dict(zip(node_ids, coords))

    # reshape tetra connectivity
    elem_nodes = np.array(elem_nodes).reshape(-1, 4)

    # compute centroids
    centroids = np.array([
        np.mean([node_map[n] for n in tet], axis=0)
        for tet in elem_nodes
    ])

    # test centroids
    element_classes_txt = os.path.splitext(args.output_msh)[0] + "_element_classes.txt"
    with open(element_classes_txt, 'w') as file:
        interior = trimesh.load(transformed_lesion_stl)
        for i,c in enumerate(centroids):
            if interior.contains([c])[0]:
                file.write('1\n')
            else:
                file.write('0\n')

    # === Identify outer surface of prostate mesh ===
    # first get the .obj surface for the combined mesh
    surface_obj = tmp_dir + "/surface.obj"
    common.getSurfaceOBJFromMSH(args.output_msh, surface_obj)
    
    prostate_surface = trimesh.load(surface_obj, force='mesh')

    if not prostate_surface.is_watertight:
        print("Warning: mesh is not watertight")

    # compute convex hull
    hull = prostate_surface.convex_hull

    # Precompute hull for fast queries
    hull_kdtree = trimesh.proximity.ProximityQuery(hull)

    # centroids and normals
    face_centroids = prostate_surface.triangles_center
    face_normals = prostate_surface.face_normals

    # distance from centroids to hull
    distances = hull_kdtree.signed_distance(face_centroids)
    distances = np.abs(distances)

    # hull proximity filter - distances less than 50th percentile are the outer surface
    outer_eps = np.percentile(distances, 30)
    inner_eps = np.percentile(distances, 70)

    outer_faces = distances < outer_eps
    inner_faces = distances > inner_eps

    selected = np.where(outer_faces)[0]
    eroded = common.erodeSelectionByArea(selected, prostate_surface.vertices, prostate_surface.faces, 0.8)

    # generate the fixed faces file from the eroded selection
    fixed_faces_filename = os.path.splitext(args.output_msh)[0] + "_fixed_faces.txt"
    with open(fixed_faces_filename, "w") as fixed_faces_file:
        for f_ind in eroded:
            face = prostate_surface.faces[f_ind]
            fixed_faces_file.write(f"3 {face[0]} {face[1]} {face[2]} {f_ind}\n")
            # print(f"Face {f_ind}: {face} \n\t{prostate_surface.vertices[face[0]]}\n\t{prostate_surface.vertices[face[1]]}\n\t{prostate_surface.vertices[face[2]]}")

    outer_patch = trimesh.Trimesh(
        vertices=prostate_surface.vertices,
        faces=prostate_surface.faces[list(eroded)],
        process=False
    )

    inner_patch = trimesh.Trimesh(
        vertices=prostate_surface.vertices,
        faces=prostate_surface.faces[list(inner_faces)],
        process=False
    )

    # === Generate nominal transform ===

    # find tunnel axis - this will be z-axis
    direction, _ = find_tunnel_axis(inner_patch, samples=500)

    inner_patch_center = inner_patch.vertices.mean(axis=0)
    dir_line = trimesh.load_path(
        np.vstack([inner_patch_center, inner_patch_center + 50 * direction])
    )

    transformed_lesion = trimesh.load(transformed_lesion_stl)
    lesion_centroid = transformed_lesion.vertices.mean(axis=0)
    
    # line from inner patch center to lesion centroid is roughly the y axis
    # approx_y_axis = inner_patch_center - lesion_centroid
    # x_axis = np.cross(approx_y_axis, direction)
    # y_axis = np.cross(direction, x_axis)
    # x_axis /= np.linalg.norm(x_axis)
    # y_axis /= np.linalg.norm(y_axis)
    # z_axis = direction / np.linalg.norm(direction)

    z_axis = direction / np.linalg.norm(direction)

    y0 = inner_patch_center - lesion_centroid
    y0 /= np.linalg.norm(y0)

    # remove any z component
    y_axis = y0 - np.dot(y0, z_axis) * z_axis
    y_axis /= np.linalg.norm(y_axis)

    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    # recompute y to ensure orthogonality
    y_axis = np.cross(z_axis, x_axis)

    R = np.zeros((3,3))
    R[:,0] = x_axis
    R[:,1] = y_axis
    R[:,2] = z_axis

    print(R)

    ## TODO: WHY IS THIS TRANSPOSE??
    from scipy.spatial.transform import Rotation as Rot
    r = Rot.from_matrix(R.T)
    eul_XYZ = np.rad2deg(r.as_euler('xyz'))

    p_des = [0,10,30]
    t = p_des - R.T @ inner_patch_center
    print(t)

    # convert translation to meters
    t /= 1000

    meshA_vis = prostate_surface.copy()
    meshA_vis.visual.face_colors = [180, 180, 180, 100]

    outer_patch.visual.face_colors = [255, 0, 0, 150]
    inner_patch.visual.face_colors = [0, 0, 255, 150]

    
    xline = trimesh.load_path(
        np.vstack([inner_patch_center, inner_patch_center + 40 * x_axis])
    )
    yline = trimesh.load_path(
        np.vstack([inner_patch_center, inner_patch_center + 40 * y_axis])
    )
    zline = trimesh.load_path(
        np.vstack([inner_patch_center, inner_patch_center + 40 * z_axis])
    )

    trimesh.Scene([
        meshA_vis,
        outer_patch,
        inner_patch,
        dir_line,
        xline,
        yline,
        zline
    ]).show(smooth=False)


    cmd = [
        "ros2",
        "launch",
        "launch/focal_lesion_sim_bridge.launch.py",
        f"config_filename:={args.config_filename}",
        f"prostate_mesh_filename:={args.output_msh}",
        # f"CT_to_VB_translation:=[{t[0]:.5f},{t[1]:.5f},{t[2]:.5f}]",
        f"CT_to_VB_translation:=[0.5,0.5,0.5]"      # for now, just put the robot far away from the tissue, let registration take care of the transforms
        # f"CT_to_VB_rotation:=[{eul_XYZ[0]:.3f},{eul_XYZ[1]:.3f},{eul_XYZ[2]:.3f}]"
    ]

    subprocess.run(cmd)



if __name__ == '__main__':
    main()