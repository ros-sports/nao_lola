# Holds Gyroscope information for a NAO V6
# Units are in rad/s
# Follows conventions here: http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html

# x is rotating right / left vertically (from robots perspective)
# - Falling right (from robot's perspective) is positive
# - Falling left (from robot's perspective) is negative
float32 x

# y is rotating over toes and heels
# - Rotating over toes is positive
# - Rotating over heels is negative
float32 y

# NOTE: This is flipped on the V5, and because of that all of gyroscopez may be inverted in a lot of codebases.
# z is rotating left / right horizontally (from robot's perspective)
# - Turning to the left is positive
# - Turning to the right is negative
float32 z