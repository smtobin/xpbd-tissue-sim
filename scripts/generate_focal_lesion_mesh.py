import numpy as np
import pymeshlab
import argparse
import os
import gmsh
import math
import trimesh
import collections
from stl import mesh

import common

parser = argparse.ArgumentParser(description='Process STL mesh files')
parser.add_argument('prostate_stl', help='Prostate surface STL file (From CT)')
parser.add_argument('lesion_stl', help='Lesion surface STL file (from CT)')
parser.add_argument('--lesion-vertices', type=int, default=100, help='Numer of target vertices for the lesion mesh')
parser.add_argument('--prostate-vertices', type=int, default=1000, help='Number of target vertices for the prostate mesh')
parser.add_argument("--lesion-translation", type=float, nargs=3, default=[0,0,0], help='Additional translation [mm] of the lesion input mesh.')
parser.add_argument("--lesion-rotation", type=float, nargs=3, default=[0,0,0], help='Additional rotation (XYZ Euler angles, deg) of the lesion input mesh.')
parser.add_argument("--lesion-scaling", type=float, nargs=3, default=[1,1,1], help='Scaling of the lesion mesh along X,Y,Z axes.')
parser.add_argument('-o', '--output-msh', default='prostate_with_lesion.msh', help='Output .msh file (optional)')

def face_key(n1, n2, n3):
    # stable identity: node IDs (NOT coordinates)
    return tuple(sorted((n1, n2, n3)))

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

    print(f"=== Processing trachea mesh ===")
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
    common.generateMSHfromSTL(processed_lesion_stl, lesion_msh)

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
        interior = trimesh.load(processed_lesion_stl)
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
    eps = np.percentile(distances, 50)
    candidate_outer = distances < eps

    outer_faces = candidate_outer
    inner_faces = ~outer_faces

    selected = np.where(outer_faces)[0]
    eroded = common.erodeSelectionByArea(selected, prostate_surface.vertices, prostate_surface.faces, 0.8)

    # generate the fixed faces file from the eroded selection
    fixed_faces_filename = os.path.splitext(args.output_msh)[0] + "_fixed_faces.txt"
    with open(fixed_faces_filename, "w") as fixed_faces_file:
        for f_ind in eroded:
            face = prostate_surface.faces[f_ind]
            fixed_faces_file.write(f"3 {face[0]} {face[1]} {face[2]} {f_ind}\n")

    outer_patch = trimesh.Trimesh(
        vertices=prostate_surface.vertices,
        faces=prostate_surface.faces[list(eroded)],
        process=False
    )

    meshA_vis = prostate_surface.copy()
    meshA_vis.visual.face_colors = [180, 180, 180, 100]

    outer_patch.visual.face_colors = [255, 0, 0, 255]

    trimesh.Scene([
        meshA_vis,
        outer_patch
    ]).show(smooth=False)



if __name__ == '__main__':
    main()