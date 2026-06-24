import numpy as np
import pymeshlab
import argparse
import os, shutil
import gmsh
import math
import trimesh
import scipy
import collections
import networkx as nx

import common

parser = argparse.ArgumentParser(description='Generate necessary files for CAO simulation')
parser.add_argument('tumor_stl', help='Tumor STL file (from CT)')
parser.add_argument('trachea_stl', help='Trachea STL file (from CT)')
parser.add_argument('--tumor-vertices', type=int, default=800, help='Number of target vertices for the tumor mesh.')
parser.add_argument('--trachea-vertices', type=int, default=1500, help='Number of target vertices for the trachea mesh.')
parser.add_argument('-o1', '--tumor-output-msh', default='tumor.msh', help='Output .msh file for the tumor (optional).')
parser.add_argument('-o2', '--trachea-output-stl', default='trachea.stl', help='Output .stl file for the trachea (optional).')
parser.add_argument('-e', '--element-classes', help='Output .txt element classes file (optional)')

def face_adjacency(F):
    edge_to_faces = collections.defaultdict(list)

    for fi, face in enumerate(F):
        for i in range(3):
            a = face[i]
            b = face[(i + 1) % 3]
            edge = tuple(sorted((a, b)))
            edge_to_faces[edge].append(fi)

    adj = [[] for _ in range(len(F))]

    for faces in edge_to_faces.values():
        if len(faces) == 2:
            f0, f1 = faces
            adj[f0].append(f1)
            adj[f1].append(f0)

    return adj

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
    adj = face_adjacency(FA)

    # find boundary
    boundary = set()
    for f in selected:
        if any(n not in selected for n in adj[f]):  # any adjacent face not in the selected region = boundary
            boundary.add(f)

    # compute centroids of faces
    centers = VA[FA].mean(axis=1)

    # multi-seeded Dijkstra for computing distance of each face from the boundary
    # (thanks ChatGPT)
    dist = np.full(len(meshA.faces), np.inf)
    for f in boundary:
        dist[f] = 0

    import heapq
    pq = [(0.0, f) for f in boundary]
    heapq.heapify(pq)

    while pq:
        d, f = heapq.heappop(pq)

        if d > dist[f]:
            continue

        for nbr in adj[f]:

            if nbr not in selected:
                continue

            w = np.linalg.norm(
                centers[f] - centers[nbr]
            )

            nd = d + w

            if nd < dist[nbr]:
                dist[nbr] = nd
                heapq.heappush(pq, (nd, nbr))


    # === Erode so that only ~5% of the original selected area remains ===
    tris = VA[FA]
    face_area = 0.5 * np.linalg.norm(
        np.cross(
            tris[:,1] - tris[:,0],
            tris[:,2] - tris[:,0]
        ),
        axis=1
    )

    # cumulative sum to get 95th percentile
    sel = np.array(list(selected))
    d = dist[sel]
    a = face_area[sel]
    order = np.argsort(d)
    d = d[order]
    a = a[order]
    cum_area = np.cumsum(a)
    cum_area /= cum_area[-1]
    threshold = d[np.searchsorted(cum_area, 0.95)]
    
    # erode the selection
    eroded = {
        f for f in selected
        if dist[f] > threshold
    }

    print(f"Eroded down to {len(eroded)} boundary faces")

    # generate the fixed faces file from the eroded selection
    fixed_faces_filename = os.path.splitext(args.output_msh)[0] + "_fixed_faces.txt"
    with open(fixed_faces_filename, "w") as fixed_faces_file:
        for f_ind in eroded:
            face = FA[f_ind]
            fixed_faces_file.write(f"3 {face[0]} {face[1]} {face[2]} {f_ind}\n")

    patch = meshA.submesh([list(eroded)], append=True)

    meshA_vis = meshA.copy()
    meshA_vis.visual.face_colors = [180, 180, 180, 100]
    meshB_vis = meshB.copy()
    meshB_vis.visual.face_colors = [200, 200, 200, 100]

    patch.visual.face_colors = [255, 0, 0, 255]

    trimesh.Scene([
        meshA_vis,
        patch,
        meshB_vis
    ]).show(smooth=False)

    shutil.rmtree(tmp_dir)

if __name__ == '__main__':
    main()