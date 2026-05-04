# MeshState.msg
# Represents the current state of a tetrahedral mesh - vertices, faces, elements.
# Each list of vertices, faces, and elements may have "gaps", i.e. elements that are no longer valid.
# These correspond to vertices/faces/elements that were deleted during the course of the simulation.
std_msgs/Header header

VerticesList vertices
FacesList faces
ElementsList elements