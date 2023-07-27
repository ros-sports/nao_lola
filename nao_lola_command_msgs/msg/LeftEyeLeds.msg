# Message identifying colors for each RGB LED in the NAO's eyes.
# Expect ranges for R, G and B are 0.0 - 1.0.
# The alpha value (A) is ignored.

# LED locations and their ids are shown below
#                          _                         #
#      /                  (_)                 \      #
#     /                                        \     #
#    /                               L0         \    #
#   |         ____              L1  ____  L7     |   #
#   |        /    \                /    \        |   #
#   |       |  **  |          L2  |  **  |  L6   |   #
#   |        \____/                \____/        |   #
#   |                           L3        L5     |   #
#    \                               L4         /    #
#     \                                        /     #
#      \                  ___                 /      #
#       \__               \_/             __/        #
#          \__                         ___/          #
#             \_______________________/              #

# For more details, refer to http://doc.aldebaran.com/2-1/family/robots/leds_robot.html#nao-v5-v4-and-v3-3

int32 L0=0
int32 L1=1
int32 L2=2
int32 L3=3
int32 L4=4
int32 L5=5
int32 L6=6
int32 L7=7
int32 NUM_LEDS=8

std_msgs/ColorRGBA[8] colors