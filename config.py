# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# configuration file
# make a backup of this file in case you do something wrong

config = {

    'MA': [11, 9, 10, 22],  # Motor Alpha Pins
    'MB': [15, 14, 18, 23],  # Motor Beta pins
    'MZ': [8, 7, 25, 24],  # Motor Z Pins

    'MA_full_step_seq': 0,  # use full step sequence instead of default half step sequence
    'MB_full_step_seq': 0,  # ~ results in more torque but half the resolution
    'MZ_full_step_seq': 1,

    'MA_dir': 1,  # change motor dir; same as switching cables around or changing pin order
    'MB_dir': 1,  # just provided for convenience reasons
    'MZ_dir': -1,  # hast to be either 1 or -1

    'distortion_correction': 1,  # either 1 or -1; necessary for some combinations of motor directions (you'll notice)

    'MA_backlash_correction': [2, 0.003],  # extra steps to correct for backlash when changing direction
    'MB_backlash_correction': [5, 0.003],  # [number of steps, delay per step]
    'MZ_backlash_correction': [0, 0],

    'MA_always_unhold': False,  # unhold immediately after movement to prevent overheating of motor coils
    'MB_always_unhold': False,  # mostly useful on zaxis as it sometimes holds position for a long time
    'MZ_always_unhold': True,

    'rB': 100,  # arm length  (mm)

    'table_offset': 0,  # degree; changes starting position of table --> rotates everything by the angle specified

    'alpha_res': 0.0862,  # degree per step (table)
    'beta_res': 0.04182,  # degree per step (arm)
    'z_res': 0.001881,  # mm per step (z-axis)

    'feed_rate': 30,  # nominal head feed rate (mm/s)
    'acceleration': 20,  # maximum head acceleration (mm/sec^2)
    'minimum_feed_rate': 1,  # slowest speed for the head to move at (mm/s)
    'junction_deviation': 0.01,  # higher numbers result in faster cornering speeds

    'a_feed_rate': 400,  # maximum feed rate for A axis (steps per second)
    'a_acceleration': 150,  # maximum A axis acceleration (steps/sec^2)
    'b_feed_rate': 350,
    'b_acceleration': 150,
    'z_feed_rate': 200,  # z is driven without acceleration currently

    'minimum_axis_feed_rate': 50,  # used in acceleration planning for a and b axis (steps/sec)

    'SegAP_buffer_length': 200,  # length of the buffer holding segments currently being calculated in SegAP

    'Default_mode': 0,  # 0=pre-calculation, 1=simultaneous threaded calculation
}
