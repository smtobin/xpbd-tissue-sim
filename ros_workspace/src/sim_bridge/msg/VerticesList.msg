# VerticesList.msg
# Represents a list of vertices with "gaps".
std_msgs/Header header

# number of valid vertices
uint32 size
# flattened vertices list - 3N x 1 array
float64[] data
# what vertices are valid (true = valid, false = invalid) - N x 1 array
bool[] valid
