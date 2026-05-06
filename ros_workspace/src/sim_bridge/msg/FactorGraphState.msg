# FactorGraphState.msg
# Contains all information needed for factor graph inference.
std_msgs/Header header

# current state of the simulation mesh
MeshState sim_mesh
# stiffness matrix at the current simulation state
SparseMatrix sim_stiffness_mat