# Contact information for a single contact pair

# Contact ID
int32 id

# Name of first geom in contact
string geom1

# Name of second geom in contact
string geom2

# Contact position in world frame
geometry_msgs/Point pos

# Contact frame (3x3 rotation matrix stored row-wise)
float64[9] frame

# Distance between geoms (negative for penetration)
float64 dist

# Contact force (6D: normal force, 2 tangential forces, 3 torques)
float64[6] force

# Normal force magnitude
float64 normal_force

# Friction force magnitude (tangential components)
float64 friction_force
