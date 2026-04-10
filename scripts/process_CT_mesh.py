import numpy as np
import pymeshlab
import argparse
import os

parser = argparse.ArgumentParser(description='Process STL mesh files')
parser.add_argument('input', help='Input STL file')
parser.add_argument('-o', '--output', help='Output STL file (optional)')

def main():
    args = parser.parse_args()

    print(f"Reading mesh from {args.input}...")

    # load mesh with MeshLab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(args.input)

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
    while (ms.current_mesh().vertex_number() > 3000):
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

    base_filename = os.path.splitext(args.input)[0]
    output_filename = base_filename + "_auto.stl"
    print(f"Saving processed mesh to {output_filename}...")
    ms.save_current_mesh(output_filename)


if __name__ == '__main__':
    main()