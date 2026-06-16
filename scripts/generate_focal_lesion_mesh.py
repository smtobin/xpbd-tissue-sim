import numpy as np
import pymeshlab
import argparse
import os
import gmsh
import math
import trimesh
import collections

parser = argparse.ArgumentParser(description='Process STL mesh files')
parser.add_argument('outside', help='Outside surface STL file')
parser.add_argument('inside', help='Embedded surface STL file')
parser.add_argument('-o', '--output-msh', help='Output .msh file (optional)')
parser.add_argument('-e', '--element-classes', help='Output .txt element classes file (optional)')

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

def face_key(n1, n2, n3):
    # stable identity: node IDs (NOT coordinates)
    return tuple(sorted((n1, n2, n3)))

def main():
    args = parser.parse_args()

    print(f"Creating unified STL mesh with outer surface from {args.outside} and inner surface from {args.inside}...")

    # load mesh with MeshLab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(args.outside)
    ms.load_new_mesh(args.inside)

    # for now, assume that the meshes are manifold and watertight
    # TODO: do some optional mesh processing to decimate

    # use boolean difference to combine the meshes
    ms.generate_boolean_difference(
        first_mesh=0,
        second_mesh=1
    )

    result_id = ms.number_meshes() - 1

    ms.set_current_mesh(result_id)
    ms.save_current_mesh("hollow.stl")

    gmsh.initialize()
    generateMSHfromSTL("hollow.stl", "hollow.msh")
    generateMSHfromSTL(args.inside, "interior.msh")

    gmsh.clear()
    gmsh.model.add("merged")

    gmsh.merge("hollow.msh")
    gmsh.merge("interior.msh")

    # gmsh.model.mesh.renumberNodes()
    # gmsh.model.mesh.renumberElements()
    gmsh.model.mesh.removeDuplicateNodes()
    gmsh.model.mesh.removeDuplicateElements()

    entities = gmsh.model.getEntities(2)

    for dim, ent_tag in entities:
        elem_types, elem_tags, _ = gmsh.model.mesh.getElements(dim, ent_tag)

        total = sum(len(t) for t in elem_tags)
        print(f"Surface {ent_tag}: {total} elements")

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
            print(tags)
            break


    element_to_surface = {}

    for dim, surf_tag in gmsh.model.getEntities(2):
        elem_types, elem_tags, _ = gmsh.model.mesh.getElements(dim, surf_tag)

        for tags in elem_tags:
            for e in tags:
                element_to_surface[int(e)] = surf_tag
    print(element_to_surface[624])
    print(element_to_surface[1115])


    

    # -------------------------------------------------------
    # Identify interface faces (shared by 2 elements)
    # -------------------------------------------------------
    interface_elements = set()

    for key, tags in face_map.items():
        if len(tags) == 2:
            interface_elements.update(tags)

    interface_elements = list(interface_elements)

    print(f"Found {len(interface_elements)} interface triangles")

    remove_by_surface = collections.defaultdict(list)

    for e in interface_elements:
        surf = element_to_surface[int(e)]
        remove_by_surface[surf].append(int(e))

    for surf, elems in remove_by_surface.items():
        print(f"Removing {len(elems)} elements from surface {surf}")
        gmsh.model.mesh.removeElements(2, surf, elems)

    # -------------------------------------------------------
    # Remove interface elements properly
    # removeElements(dim, elementTags, elementType)
    # triangle type = 2
    # -------------------------------------------------------
    # gmsh.model.mesh.removeElements(2, 2, interface_elements_int)

    # -------------------------------------------------------
    # Cleanup mesh
    # -------------------------------------------------------
    # gmsh.model.mesh.reclassifyNodes()

    gmsh.write("merged.msh")

    gmsh.clear()

    gmsh.open("merged.msh")
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
    print(centroids)

    # test centroids
    with open('classes.txt', 'a') as file:
        interior = trimesh.load(args.inside)
        for i,c in enumerate(centroids):
            if interior.contains([c])[0]:
                file.write('1\n')
            else:
                file.write('0\n')

    gmsh.finalize()



if __name__ == '__main__':
    main()