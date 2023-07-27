# Message to specify positions for the NAO's motor joints.
#
# Each joint is uniquely identified by its index (See JointIndexes.msg)
#
# The two arrays in this message should have the same size, where the first item
# in indexes, corresponds to the first item in positions, etc.

uint8[] indexes  # See JointIndexes.msg (eg. JointIndexes::HEADYAW)
float32[] positions # radians