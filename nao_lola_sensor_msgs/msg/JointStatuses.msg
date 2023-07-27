# Temperature status enums, computed accordingly to the temperature limitation to protect the motors.
int32 STATUS_NORMAL=0          # normal
int32 STATUS_HOT=1             # high, start to reduce stiffness
int32 STATUS_VERY_HOT=2        # very hot, stiffness reduced over 30%
int32 STATUS_CRITICALLY_HOT=3  # critically hot, stiffness is set to 0

int32[25] statuses  # Status codes, in order of JointIndexes.msg