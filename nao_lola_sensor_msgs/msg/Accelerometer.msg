# Holds Accelerometer information for a NAO V6
# Units are in m/s^2
# Follows conventions here: http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html
# Note that gravity is measured by an accelerometer.
# When standing, the robot will measure 9.8m/s^2 POSITIVE in the z-direction, since its 
# receiving an acceleration upwards from the floor when compared to a freefall state.

# x is moving forwards / backwards (from robots perspective)
# - Forwards is positive
# - Backwards is negative
float32 x

# y is moving left / right (from robot's perspective)
# - Left is positive
# - Right is negative
float32 y

# z is moving up / down
# - Standing straight is negative
# - Upside down is positive
float32 z