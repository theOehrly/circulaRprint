# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# different classes that are used for data handling

class GCommand:
    def __init__(self, gtype, setup, gcode, index):
        self.gtype = gtype  # g-command type
        self.setup = setup  # True/False : contains setup information instead of movement information
        self.gcode = gcode  # gcode; example format : {'Z': '-1.000000', 'Y': '25.513603 ', 'X': '10.615510 '}
        self.index = index  # gcommands get a index number in the order they are loaded in
        self.status = None  # additional status info
        self.path = list()  # list of steps PathSegment class objects
        self.x = None       # x pos after movement
        self.y = None       # y pos after movement
        self.z = None
        self.info = None    # any additional info

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class PathSegment:
    def __init__(self, a_steps, b_steps, z_steps, x, y, z, alpha, beta, gcommand, index):

        self.a_steps = a_steps
        self.b_steps = b_steps
        self.z_steps = z_steps

        self.x = x  # x position after movement
        self.y = y  # y position after movement
        self.z = z

        self.alpha = alpha  # axes angles after movement
        self.beta = beta

        self.gcommand = gcommand  # associated gcommand object
        self.index = index  # index number specifies order of path segments

        self.a_timing = list()  # per step timing a
        self.b_timing = list()  # per step timing b
        self.z_delay = float()  # delay per step on z axis

        self.entry_speed_sqr = 0
        self.max_entry_speed_sqr = 0
        self.max_junction_speed_sqr = 0
        self.nominal_speed = 0
        self.acceleration = 0
        self.distance = 0

        self.x_unit_vector = 0
        self.y_unit_vector = 0

