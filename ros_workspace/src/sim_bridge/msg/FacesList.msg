# FacesList.msg
# Represents a list of triangular faces with "gaps".
# Faces are specified in terms of the vertex indices that make up the face.
std_msgs/Header header

# number of valid faces
uint32 size
# flattened faces list - 3N x 1 array
int32[] data
# a list indices that "invalid" - i.e. correspond to faces that no longer exist in the mesh
int32[] invalid_indices