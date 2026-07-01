std_msgs/Header header
# the query point
sim_bridge/PointOnFace point_on_face

# 3x3 local compliance matrix at the point on the face_barys
# assumes row-major ordering!
float32[] compliance_mat