# VerticesList.msg
# Represents a list of vertices with "gaps".
std_msgs/Header header

# number of valid vertices
uint32 size
# flattened vertices list - 3N x 1 array
float64[] data
# a list indices that "invalid" - i.e. correspond to vertices that no longer exist in the mesh
int32[] invalid_indices
