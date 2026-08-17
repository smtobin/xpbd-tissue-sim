# ArmArmCollision.msg
# Represents a detected collision between two Virtuoso arms

# Whether or not there is a collision
# 0 = no collision
# 1 = collision
int8 collision

# Collision location on arm 1, expressed in terms of relative arc length
# i.e. 0 = base of arm, 1 = tip of arm, >1 = tool
float32 arm1_location

# Collision location on arm 2, expressed in terms of relative arc length
float32 arm2_location