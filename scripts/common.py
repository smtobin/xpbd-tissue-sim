import numpy as np
import pymeshlab
import argparse
import os
import gmsh
import math
import trimesh
import collections
import meshio

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