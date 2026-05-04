# FacesList.msg
# Represents a list of triangular faces with "gaps".
# Faces are specified in terms of the vertex indices that make up the face.
std_msgs/Header header

# number of valid faces
uint32 size
# flattened faces list - 3N x 1 array
int32[] data
# what faces are valid (true = valid, false = invalid) - N x 1 array
bool[] valid