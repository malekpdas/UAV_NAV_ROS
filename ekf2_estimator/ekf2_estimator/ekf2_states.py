import numpy as np

# State Index Definition
# x = [pos_ned(3), vel_ned(3), quat(4), gyro_bias(3), accel_bias(3), mag_I(3), mag_B(3), wind_ne(2)]
# Total: 24 states

IX_POS_N = 0
IX_POS_E = 1
IX_POS_D = 2
IX_VEL_N = 3
IX_VEL_E = 4
IX_VEL_D = 5
IX_QUAT_W = 6
IX_QUAT_X = 7
IX_QUAT_Y = 8
IX_QUAT_Z = 9
IX_GB_X = 10
IX_GB_Y = 11
IX_GB_Z = 12
IX_AB_X = 13
IX_AB_Y = 14
IX_AB_Z = 15
IX_MAG_I_X = 16
IX_MAG_I_Y = 17
IX_MAG_I_Z = 18
IX_MAG_B_X = 19
IX_MAG_B_Y = 20
IX_MAG_B_Z = 21
IX_WIND_N = 22
IX_WIND_E = 23

NUM_STATES = 24

def init_state():
    x = np.zeros(NUM_STATES)
    x[IX_QUAT_W] = 1.0
    return x
