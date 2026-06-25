import numpy as np
import pymeshlab
import argparse
import os
import gmsh
import math
import trimesh
import collections
import meshio

def rotation_matrix_to_euler_zyx(R):
    """
    Convert rotation matrix to intrinsic ZYX Euler angles.

    Returns:
        yaw   (about Z)
        pitch (about Y)
        roll  (about X)
    """

    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)

    singular = sy < 1e-6

    if not singular:
        yaw   = np.arctan2(R[1,0], R[0,0])
        pitch = np.arctan2(-R[2,0], sy)
        roll  = np.arctan2(R[2,1], R[2,2])
    else:
        # Gimbal lock
        yaw   = np.arctan2(-R[0,1], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        roll  = 0.0

    return np.array([yaw, pitch, roll])

def rotation_matrix_to_euler_xyz(R):
    """
    Convert a 3x3 rotation matrix to XYZ Euler angles.

    Convention:
        R = Rx(alpha) @ Ry(beta) @ Rz(gamma)

    Returns:
        alpha : rotation about X (roll)
        beta  : rotation about Y (pitch)
        gamma : rotation about Z (yaw)

    Angles are returned in radians.
    """
    R = np.asarray(R, dtype=float)

    if R.shape != (3, 3):
        raise ValueError("R must be a 3x3 matrix")

    # Detect gimbal lock
    if abs(R[0, 2]) < 1.0 - 1e-8:
        beta = np.arcsin(R[0, 2])

        alpha = np.arctan2(-R[1, 2], R[2, 2])
        gamma = np.arctan2(-R[0, 1], R[0, 0])

    else:
        # Gimbal lock
        beta = np.pi / 2 * np.sign(R[0, 2])

        alpha = 0.0

        if R[0, 2] > 0:
            gamma = np.arctan2(R[1, 0], R[1, 1])
        else:
            gamma = np.arctan2(R[1, 0], R[1, 1])

    return alpha, beta, gamma


def write_obj(filename, vertices, faces):

    with open(filename, "w") as obj_file:

        # write vertices
        for v in vertices:
            obj_file.write(f"v {v[0]} {v[1]} {v[2]}\n")

        # write faces (OBJ is 1-based indexing)
        for f in faces:
            obj_file.write(f"f {f[0] + 1} {f[1] + 1} {f[2] + 1}\n")

def generateMSHfromSTL(input_stl, output_msh):
    gmsh.clear()
    gmsh.model.add("mesh")

    gmsh.merge(input_stl)

    gmsh.model.mesh.classifySurfaces(
        40 * math.pi / 180,
        True,
        False,
        math.pi
    )

    surfs = gmsh.model.getEntities(2)

    sl = gmsh.model.geo.addSurfaceLoop([s[1] for s in surfs])
    v = gmsh.model.geo.addVolume([sl])

    gmsh.model.geo.synchronize()

    gmsh.model.mesh.generate(3)
    gmsh.write(output_msh)
    gmsh.clear()

def getSurfaceOBJFromMSH(input_msh, output_file):

    gmsh.open(input_msh)

    entities = gmsh.model.getEntities()

    vertices = []
    faces = []
    elements = []

    for dim, tag in entities:

        nodeTags, nodeCoords, _ = gmsh.model.mesh.getNodes(dim, tag)

        nodeTags = np.array(nodeTags)
        nodeCoords = np.array(nodeCoords).reshape(-1, 3)

        # map node tag → local index
        tag_to_idx = {t: i for i, t in enumerate(nodeTags)}

        elemTypes, elemTags, elemNodeTags = gmsh.model.mesh.getElements(dim, tag)

        for etype, enodes in zip(elemTypes, elemNodeTags):

            enodes = np.array(enodes)

            # tetrahedra
            if etype == 4:
                tets = enodes.reshape(-1, 4)
                elements.extend((tets - 1).tolist())

            # triangles
            elif etype == 2:
                tris = enodes.reshape(-1, 3)
                faces.extend((tris - 1).tolist())

        vertices.extend(nodeCoords.tolist())

    gmsh.finalize()

    vertices = np.array(vertices)
    faces = np.array(faces)

    write_obj(output_file, vertices, faces)
    

def decimateToTargetNumberOfVertices(input_stl, target_vertices, output_stl):
    print(f"Reading mesh from {input_stl}...")

    # load mesh with MeshLab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(input_stl)

    print(f"Keeping only largest component...")
    # Remove small disconnected components (keep only the largest)
    ms.compute_selection_by_small_disconnected_components_per_face(nbfaceratio=0.9)
    ms.meshing_remove_selected_faces()

    # Initially remove all T-vertices and non-manifold faces
    print(f"Removing T-vertices and non-manifold edges...")
    ms.meshing_remove_t_vertices()
    ms.meshing_repair_non_manifold_edges()

    # Decimate the mesh, reducing the number of faces by half each time until <800 vertices are left
    print(f"Decimating mesh...")
    while (ms.current_mesh().vertex_number() > target_vertices):
        print(f"  Current number of vertices: {ms.current_mesh().vertex_number()}")
        ms.meshing_decimation_quadric_edge_collapse(targetperc=0.5, qualitythr=0.5, autoclean=True)

    print(f"Final number of vertices: {ms.current_mesh().vertex_number()}")
    print(f"Final number of faces: {ms.current_mesh().face_number()}")

    # Clean the mesh (remove non-manifold stuff mainly)
    # Select self-intersecting faces
    print(f"Cleaning mesh...")
    print(f"  Selecting self-intersecting faces...")
    ms.compute_selection_by_self_intersections_per_face()

    # Select non-manifold edges/vertices
    print(f"  Selecting non-manifold edges and vertices...")
    ms.compute_selection_by_non_manifold_edges_per_face()
    ms.compute_selection_by_non_manifold_per_vertex()

    print(f"  Dilating selection...")
    # Dilate the selection to get an area around the problematic faces
    # The updateradius parameter controls how far to expand (in number of face rings)
    ms.apply_selection_dilatation()  # Adjust this value as needed

    print(f"  Removing selected faces...")
    # Delete the selected faces
    ms.meshing_remove_selected_faces()

    print(f"  Removing unreferenced vertices...")
    # Remove unreferenced vertices
    ms.meshing_remove_unreferenced_vertices()

    

    # Ensure consistent face orientation
    ms.meshing_re_orient_faces_coherently()

    # Remove small disconnected components (keep only the largest)
    # sometimes there are small internal voids in the mesh...need to remove these
    print(f"Keeping only largest component (again)...")
    ms.compute_selection_by_small_disconnected_components_per_face(nbfaceratio=0.9)
    ms.meshing_remove_selected_faces()

    # Select self-intersecting faces
    print(f"Cleaning mesh...")
    print(f"  Selecting self-intersecting faces...")
    ms.compute_selection_by_self_intersections_per_face()
    print(f"  Removing selected faces...")
    # Delete the selected faces
    ms.meshing_remove_selected_faces()
    print(f"  Closing holes...")
    # Close holes
    ms.meshing_close_holes(maxholesize=30)  # Adjust maxholesize as needed

    # Select non-manifold edges/vertices
    print(f"  Selecting non-manifold edges and vertices...")
    ms.compute_selection_by_non_manifold_edges_per_face()
    ms.compute_selection_by_non_manifold_per_vertex()

    print(f"  Dilating selection...")
    # Dilate the selection to get an area around the problematic faces
    # The updateradius parameter controls how far to expand (in number of face rings)
    ms.apply_selection_dilatation()  # Adjust this value as needed

    print(f"  Closing holes...")
    # Close holes
    ms.meshing_close_holes(maxholesize=30)  # Adjust maxholesize as needed

    print(f"  Removing unreferenced vertices...")
    # Remove unreferenced vertices
    ms.meshing_remove_unreferenced_vertices()

    if output_stl is not None:
        ms.save_current_mesh(output_stl)
    else:
        base_filename = os.path.splitext(input_stl)[0]
        output_stl = base_filename + f"_{target_vertices}.stl"

    print(f"Saving processed mesh to {output_stl}...")
    ms.save_current_mesh(output_stl)

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

def erodeSelectionByArea(selected, vertices_mat, faces_mat, percentile):
    adj = face_adjacency(faces_mat)

    # find boundary
    boundary = set()
    for f in selected:
        if any(n not in selected for n in adj[f]):  # any adjacent face not in the selected region = boundary
            boundary.add(f)

    # compute centroids of faces
    centers = vertices_mat[faces_mat].mean(axis=1)

    # multi-seeded Dijkstra for computing distance of each face from the boundary
    # (thanks ChatGPT)
    dist = np.full(len(faces_mat), np.inf)
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
    tris = vertices_mat[faces_mat]
    face_area = 0.5 * np.linalg.norm(
        np.cross(
            tris[:,1] - tris[:,0],
            tris[:,2] - tris[:,0]
        ),
        axis=1
    )

    # cumulative sum to get the specified percentile
    sel = np.array(list(selected))
    d = dist[sel]
    a = face_area[sel]
    order = np.argsort(d)
    d = d[order]
    a = a[order]
    cum_area = np.cumsum(a)
    cum_area /= cum_area[-1]
    threshold = d[np.searchsorted(cum_area, percentile)]
    
    # erode the selection
    eroded = {
        f for f in selected
        if dist[f] > threshold
    }

    print(f"Eroded down to {len(eroded)} boundary faces")

    return eroded

    