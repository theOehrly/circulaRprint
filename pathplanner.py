# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# calculates motor steps, angles and x-y-position from gcode
#
# If you want to now more about how the maths work
# take a look at the separate documentation for it


from math import atan, asin, sin, cos, radians, pi, sqrt
from mathfunc import sign
from datastructures import PathSegment

from copy import copy


class PathPlanner:
    # Does interpolation and path calculations and returns PathSegments containing
    # lists of motor steps for each axis.
    #
    # All calculations are based on the movement of the beta axis (arm).
    # For every movement (except special cases) it is first calculated to which position the
    # beta arm needs to move.
    # This movement is then executed in steps of the arms minimum resolution. For each step
    # of movement, the current X and Y coordinates are calculated. From that it is finally
    # calculated to which position the alpha axis needs to move (table).
    #
    # This approach is used as the printers resolution in millimetres varies depending on the
    # current position and its distance to the centre. The further away we get, the lower the
    # alpha axis' resolution gets. The beta axis' resolution stays the same all the time as
    # the distance to it's pivot point does never change (the arms length is fixed).
    #
    # Moving in fixed cartesian millimetre steps (i.e. +0.05mm on x or something like that)
    # would have made the maths quite a bit easier and more straight forward. The disadvantage
    # is that on the one hand the printers high resolution at the centre can not be used. On the
    # other hand getting further outside, the code would have made more and more iterations that
    # result in no movement at all as the printer is just not as accurate at this position.
    # Choosing a fixed step size would always have meant choosing a trade-off between lost
    # accuracy at the center and unnecessary calculations further outside.
    # Also I hope that avoiding unnecessary calculations may compensate for the maybe more complex
    # and time consuming calculations of this approach.
    #
    # Radians are used for all calculations instead of degrees.
    #
    # Closest position to target is reached if the distance to target is smaller than 0.5 * resolution
    # Example: if distance to target is -0.7 * res, moving one more step will result in +0.3 * res --> closer
    #
    # Limitation on circular interpolation: The planner currently only supports circle segments covering
    # up to 180 degrees of a circle. If a bigger segment (for example a full circle) is desired, it has to
    # be split up into multiple G02/G03 commands prior to handing it to this class!
    #
    # For even further information on the mathematics involved, take a look at the separate
    # documentation for it. It provides a more in-depth explanation and graphics for visualization.
    # Also all steps for deriving the formulas are written out there.
    #
    # Feel free to contact me for questions or suggestions.

    def __init__(self, config, source):
        # load from config
        self.TABLE_OFFSET = radians(float(config['table_offset']))
        self.rB = float(config['rB'])  # radius beta (arm length)

        self.ALPHA_RES = float(radians(config['alpha_res']))  # axis resolutions in radians per step
        self.BETA_RES = float(radians(config['beta_res']))
        self.Z_RES = float(config['z_res'])

        self.x = float(0)  # real current position in cartesian coordinates given in millimetres
        self.y = float(0)  # table center is (0|0)
        self.z = float(0)

        self.DISTORTION_CORRECTION = int(config['distortion_correction'])

        # current position of arm and table (radians)
        # alpha may not exceed +/-pi (therefore always use self.limit_alpha)
        self.alpha = self.limit_alpha(self.TABLE_OFFSET)
        self.beta = float(0)  #
        # gamma is the table angle before correction of beta offset (only stored for rare situations)
        self.gamma = float(0)

        self.current_gcommand = None  # the gcommand currently being processed

        self.segment_index = 0  # segments get a index number

        self.source = source  # class which provides new gcommands

        # debug setting: this is enabled if rprint is run through debugout.py
        # it then enables the planner to send position data for visualization
        self.DEBUGOUT = None
        if 'debugout' in config:
            self.DEBUGOUT = config['debugout']
            self.DEBUGOUT.update_status(1)

        self.segment_buffer = list()

    def serve_next(self):
        if self.segment_buffer:
            return self.segment_buffer.pop(0)

        else:
            path = None
            while not path:
                self.current_gcommand = self.source.serve_next()
                if not self.current_gcommand:
                    return None

                path = self.process(self.current_gcommand)

            self.segment_buffer = copy(path)  # create copy so deleting this buffer doesn't delete path in GCommand
            self.current_gcommand.path = path
            self.current_gcommand.set_position(self.x, self.y, self.z)

            return self.segment_buffer.pop(0)

    def process(self, gcommand):
        # takes a gcommand and hands it to the appropriate function for calculating a path of motor steps
        path = None
        if gcommand.gtype in ('G00', 'G01'):
            # G00/G01 may only contain Z axis data so we need to check for x/y
            if {'X', 'Y'}.issubset(gcommand.gcode.keys()):
                x = float(gcommand.gcode['X'])
                y = float(gcommand.gcode['Y'])
                path = self.linear_process(x, y)

            elif 'Z' in gcommand.gcode.keys():
                z = float(gcommand.gcode['Z'])
                path = self.move_z(z)

        elif gcommand.gtype in ('G02', 'G03'):
            x = float(gcommand.gcode['X'])
            y = float(gcommand.gcode['Y'])
            i = float(gcommand.gcode['I'])
            j = float(gcommand.gcode['J'])
            path = self.arc_process(x, y, i, j, gcommand.gtype)

        return path

    def move_z(self, z):
        # calculate number of steps to move
        z_dif = z - self.z
        z_steps = int(round(z_dif / self.Z_RES, 0))

        # set current z position
        self.z += z_steps * self.Z_RES

        # create and return path segment
        self.segment_index += 1
        return [PathSegment(0, 0, z_steps, self.x, self.y, self.z, self.alpha, self.beta, self.current_gcommand,
                            self.segment_index)]

    def linear_process(self, x_e, y_e):
        # prepares for linear interpolation by calculating necessary data
        #
        # the line is defined using a vector equation
        # dv = directional vector ; pv = positional vector
        #
        # First the vectors are calculated, after that the line's closest point to the center
        # is calculated. The beta arm needs to change direction at this point. Therefore, if
        # it is between start and end point, the line needs to be interpolated in two steps.

        dv_x = x_e - self.x
        dv_y = y_e - self.y
        pv_x = self.x
        pv_y = self.y

        # get minimum distance to center and position with minimum distance
        try:
            q = (-dv_x * pv_x - dv_y * pv_y) / (dv_x ** 2 + dv_y ** 2)
        except ZeroDivisionError:
            q = 0
        min_d_x = pv_x + q * dv_x
        min_d_y = pv_y + q * dv_y
        min_d = sqrt(min_d_x ** 2 + min_d_y ** 2)

        # check whether minimum distance is between start and end point
        # if so, we need to interpolate start --> min_d  and   min_d --> end
        # the lists of path segments are added together and returned as one list
        # if if x_e and y_e are equal to the current position, path is returned empty
        path = list()
        if x_e != self.x:
            if (min_d_x != self.x) and (min_d_x != x_e):
                if self.x < min_d_x < x_e or x_e < min_d_x < self.x:
                    path = self.linear_move(min_d_x, min_d_y, pv_x, pv_y, dv_x, dv_y, min_d)
                    path2 = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)
                    path += path2
                else:
                    path = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)
            else:
                path = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)
        elif y_e != self.y:
            if (min_d_y != self.y) and (min_d_y != y_e):
                if self.y < min_d_y < y_e or y_e < min_d_y < self.y:
                    path = self.linear_move(min_d_x, min_d_y, pv_x, pv_y, dv_x, dv_y, min_d)
                    path2 = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)
                    path += path2
                else:
                    path = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)
            else:
                path = self.linear_move(x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d)

        return path

    def linear_move(self, x_e, y_e, pv_x, pv_y, dv_x, dv_y, min_d):
        # calculates end position of beta arm movement and direction of movement

        d = sqrt(x_e ** 2 + y_e ** 2)  # distance to center at end position
        beta_e = 2 * asin(d / (2 * self.rB))  # value of beta at end position

        dir_beta = sign(beta_e - self.beta)  # direction of beta movement

        if abs(beta_e - self.beta) <= 0.5 * self.BETA_RES:
            # if true, beta is already as close to it's target position as possible
            # although we don't need to move the beta arm, we might still need to move the table (alpha)
            path = list()
            steps_alpha, dir_alpha = self.calc_alpha(x_e, y_e)
            self.x = x_e
            self.y = y_e
            if steps_alpha != 0:
                # alpha needs to move
                self.segment_index += 1
                path.append(PathSegment(dir_alpha * steps_alpha, 0, 0, self.x, self.y, self.z,
                                        self.alpha, self.beta, self.current_gcommand, self.segment_index))
            return path

        else:
            # beta needs to move
            return self.linear_move_beta(beta_e, dir_beta, min_d, x_e, y_e, pv_x, pv_y, dv_x, dv_y)

    def linear_move_beta(self, beta_e, dir_beta, min_d, x_e, y_e, pv_x, pv_y, dv_x, dv_y):
        # beta is moved in steps of it minimum resolution
        # x, y and alpha are calculated for every iteration
        path = list()

        last_iteration = False
        while True:
            # move beta arm one res step per iteration
            self.beta = self.beta + (self.BETA_RES * dir_beta)

            # check whether this is the last iteration of the loop
            if abs(beta_e - self.beta) <= 0.5 * self.BETA_RES:
                last_iteration = True

            # current distance of beta arm from center
            d_c = 2 * self.rB * sin(self.beta / 2)

            if not last_iteration:
                if d_c < min_d:  # calc_xy_linear fails if this is true
                    d_c = min_d  # may be caused by very slight calculation errors
                # get current x/y position
                self.x, self.y = self.calc_xy_linear(x_e, y_e, pv_x, pv_y, dv_x, dv_y, d_c)
            else:
                # compensate for small calculation errors on the last go by simply using the target values
                # as current position; seems to be fine
                self.x = x_e
                self.y = y_e

            if self.DEBUGOUT:  # dev stuff
                self.DEBUGOUT.update_position(self.x, self.y, 'linear')

            # calculate alpha position
            steps_alpha, dir_alpha = self.calc_alpha(self.x, self.y)

            # add to path
            self.segment_index += 1
            path_segment = PathSegment(dir_alpha * steps_alpha, dir_beta, 0, self.x, self.y, self.z,
                                       self.alpha, self.beta, self.current_gcommand, self.segment_index)
            path.append(path_segment)

            # checking whether to continue loop or not
            if last_iteration:
                return path

    def calc_xy_linear(self, x_e, y_e, pv_x, pv_y, dv_x, dv_y, d_c):
        # The distance between the beta arm and the center of the table can be put in relation
        # to x and y using the following equation:  d = sqrt(x**2 + y**2) [pythagorean theorem]
        # As x and y are unknown, y is replaced with the line's equation. Y is now defined in relation to x.
        #   d = sqrt(x**2 + f(x)**2)
        # The equation now has only one unknown variable and can be solved for x.
        # Transforming the it results in a quadratic equation:
        # f(x) = ax**2 + bx + c    where  a, b and c are...

        a = dv_x ** 2 + dv_y ** 2
        b = 2 * pv_x * dv_x + 2 * pv_y * dv_y
        c = pv_x ** 2 + pv_y ** 2 - d_c ** 2

        # now the equation is solved using the quadratic formula
        try:
            t1 = (-b + sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            t2 = (-b - sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        except ValueError:
            t1 = 1
            t2 = 1

        # calculate y from x
        x_c1 = pv_x + t1 * dv_x
        x_c2 = pv_x + t2 * dv_x

        # decide which x/y pair is the correct on (a quadratic equation has (up to) two solutions)
        # the dev has no idea how to do it with less if-elif-else....sorry
        # this mess checks which pair of x/y values is the correct one
        if x_c1 != x_c2:
            if (x_e <= x_c1 <= self.x or x_e >= x_c1 >= self.x) and (
                                x_e <= x_c2 <= self.x or x_e >= x_c2 >= self.x):
                if abs(x_c1 - self.x) < abs(x_c2 - self.x):
                    x_c = x_c1
                    y_c = pv_y + t1 * dv_y
                elif abs(x_c1 - self.x) > abs(x_c2 - self.x):
                    x_c = x_c2
                    y_c = pv_y + t2 * dv_y
                else:
                    raise ShouldNotBePossibleError
            elif x_e <= x_c1 <= self.x or x_e >= x_c1 >= self.x:
                x_c = x_c1
                y_c = pv_y + t1 * dv_y
            elif x_e <= x_c2 <= self.x or x_e >= x_c2 >= self.x:
                x_c = x_c2
                y_c = pv_y + t2 * dv_y
            else:
                if abs(x_c1 - self.x) < abs(x_c2 - self.x):
                    x_c = x_c1
                    y_c = pv_y + t1 * dv_y
                elif abs(x_c1 - self.x) > abs(x_c2 - self.x):
                    x_c = x_c2
                    y_c = pv_y + t2 * dv_y
                else:
                    raise ShouldNotBePossibleError

        else:
            x_c = x_c1  # which x value doesn't matter in that case anyway
            y_c1 = pv_y + t1 * dv_y
            y_c2 = pv_y + t2 * dv_y
            if y_c1 != y_c2:
                if (y_e <= y_c1 <= self.y or y_e >= y_c1 >= self.y) and (
                                    y_e <= y_c2 <= self.y or y_e >= y_c2 >= self.y):
                    if abs(y_c1 - self.y) < abs(y_c2 - self.y):
                        y_c = y_c1
                    elif abs(y_c1 - self.y) > abs(y_c2 - self.y):
                        y_c = y_c2
                    else:
                        raise ShouldNotBePossibleError
                elif y_e <= y_c1 <= self.y or y_e >= y_c1 >= self.y:
                    y_c = y_c1
                elif y_e <= y_c2 <= self.y or y_e >= y_c2 >= self.y:
                    y_c = y_c2
                else:
                    if abs(y_c1 - self.y) < abs(y_c2 - self.y):
                        y_c = y_c1
                    elif abs(y_c1 - self.y) > abs(y_c2 - self.y):
                        y_c = y_c2
                    else:
                        raise ShouldNotBePossibleError
            else:
                y_c = y_c1  # Now nothing matters anymore

        return x_c, y_c

    def arc_process(self, x_e, y_e, i, j, mv_dir):
        # prepares for circular interpolation and splits circle into segments
        # Currently circular interpolation is limited to circle segments covering
        # 180 degrees of a full circle at maximum. Larger segments need to be split
        # up into multiple G02/G03 commands prior to handing them to this class!
        #
        # This code works with the equation for a half circle:
        #   f(x) = sqrt(r^2 + (x-m)^2) + n
        #
        # A full circle would need to be split into four segments. First into two segments
        # as the equation used only defines the upper OR lower half of a circle (+f(x) or -f(x)).
        # Secondly these segments are both split again at the points closest to and furthest away
        # from the table's center, as the beta arm needs to change direction at these points.
        # Although full circles are not supported currently, these split points also apply to
        # segments of a circle.

        path = list()

        m = self.x + i  # center coordinates of the circle
        n = self.y + j

        r = sqrt((m - self.x) ** 2 + (n - self.y) ** 2)  # radius

        # if the circle's center point is the same as the table's center point, only alpha needs to move.
        # returns path empty if alpha also does not need to move
        if m == 0 and n == 0:
            steps_alpha, dir_alpha = self.calc_alpha(x_e, y_e)

            self.x = x_e
            self.y = y_e

            if steps_alpha != 0:
                self.segment_index += 1
                path.append(PathSegment(dir_alpha * steps_alpha, 0, 0, self.x, self.y, self.z,
                                        self.alpha, self.beta, self.current_gcommand, self.segment_index))

            return path

        # the beta axis needs to move
        else:
            # Calculate points closest to and furthest away from table center and corresponding min/max dist.
            # Depending on the position on the table each of these calculation can either return the closest
            # or the furthes point, therefore further checks are necessary.
            p1_x = m + m * (1 / sqrt(m ** 2 + n ** 2)) * r
            p1_y = n + n * (1 / sqrt(m ** 2 + n ** 2)) * r
            p2_x = m - m * (1 / sqrt(m ** 2 + n ** 2)) * r
            p2_y = n - n * (1 / sqrt(m ** 2 + n ** 2)) * r

            d_1 = sqrt(p1_x ** 2 + p1_y ** 2)
            d_2 = sqrt(p2_x ** 2 + p2_y ** 2)

            # Determine which is maximum and minimum distance from table center.
            min_d = min(d_1, d_2)

            # Determine upper and lower point at which the beta arm needs to change direction.
            # The upper point is on the positive circle half, the lower point on the negative circle half
            if not p1_y == p2_y:
                if p1_y > p2_y:
                    p_up_y = p1_y
                    p_up_x = p1_x
                    p_low_y = p2_y
                    p_low_x = p2_x
                else:
                    p_up_y = p2_y
                    p_up_x = p2_x
                    p_low_y = p1_y
                    p_low_x = p1_x
            else:
                # which one is which is irrelevant
                p_up_y = p1_y
                p_up_x = p1_x
                p_low_y = p2_y
                p_low_x = p2_x

            # The circle is devided into four segments, c_segs contains the split points:
            # (m +-r, n) are necessary as f(x) only defines a half circle; they split top and bottom half
            # (p*_x, p*_y) are necessary as the beta arm needs to change direction at these points
            # The code checks where the start and end points fit in and then iterates from start over the split points
            # until it reaches the end point. Per iteration, interpolation from the current position to the next point
            # is performed.
            # Taking the list times three ensures that we don't need to loop over to the beginning while iterating.
            # The list order is changed for depending on direction of movement.

            if mv_dir == 'G02':
                c_segs = [(m - r, n), (p_up_x, p_up_y), (m + r, n), (p_low_x, p_low_y)] * 3
            else:
                c_segs = [(m + r, n), (p_up_x, p_up_y), (m - r, n), (p_low_x, p_low_y)] * 3

            # find position of start and endpoint in c_segs
            #
            # self.x/self.y are the starting point's coordinates
            if not m - r <= self.x <= m + r:
                # because of slight inaccuracies it can happen that self.x is not on/in the circle
                # in that case it is just set to the min/max possible value
                if abs(self.x - (m - r)) < abs(x_e - (m + r)):
                    self.x = m - r
                else:
                    self.x = m + r

            for i in range(7):  # starting point
                # check if current x is between the current and the next split point
                if c_segs[i][0] >= self.x >= c_segs[i + 1][0] or c_segs[i][0] <= self.x <= c_segs[i + 1][0]:
                    if i in (0, 1, 4, 5):  # +f(x) / segment is upper circle half
                        # n is the circle centre's y-coordinate; the current segment is on the upper
                        # half of the circle, therefore self.y also needs to be above the circle's centre
                        # else the current point does not fit in at this position; continue searching...
                        if self.y >= n:
                            i_start = i + 1
                            break
                    elif i in (2, 3, 6, 7):  # -f(x) / segment is lower circle half
                        if self.y < n:
                            i_start = i + 1
                            break

            if not m - r <= x_e <= m + r:
                # because of slight inaccuracies it can happen that x_e is not on/in the circle
                # in that case it is just set to the min/max possible value
                if abs(x_e - (m - r)) < abs(x_e - (m + r)):
                    x_e = m - r
                else:
                    x_e = m + r

            # look were the endpoint fits in; basically same as for starting point above
            for i in range(i_start - 1, 11):  # endpoint
                if c_segs[i][0] >= x_e >= c_segs[i + 1][0] or c_segs[i][0] <= x_e <= c_segs[i + 1][0]:
                    if i in (0, 1, 4, 5, 8, 9):  # +f(x)
                        if y_e >= n:
                            i_end = i + 1
                            break
                    elif i in (2, 3, 6, 7, 10, 11):  # -f(x)
                        if y_e < n:
                            i_end = i + 1
                            break

            # pos_neg indicates whether a segment is on the upper (pos) or lower (neg) half of the circle.
            # this value is passed on to be used later for calculating the current x and y coordinates
            pos_neg = [-1, 1, 1, -1] * 3

            # do interpolation for each circle segment
            for i in range(i_start, i_end):
                path += self.arc_move_beta(c_segs[i][0], c_segs[i][1], m, n, r, min_d, pos_neg[i])

            # do interpolation to end position
            path += self.arc_move_beta(x_e, y_e, m, n, r, min_d, pos_neg[i + 1])

        return path

    def arc_move_beta(self, x_e, y_e, m, n, r, min_d, pos_neg):
        path = []

        if self.x == x_e and self.y == y_e:
            # no movement necessary
            return path

        # calculate end position of beta arm movement (and direction)
        d = sqrt(x_e ** 2 + y_e ** 2)
        beta_e = 2 * asin(d / (2 * self.rB))

        dir_beta = sign(beta_e - self.beta)

        dir_x = sign(x_e - self.x)

        last_iteration = False
        while True:
            self.beta = self.beta + (self.BETA_RES * dir_beta)  # move beta arm one res step per iteration

            # check whether this is the last iteration of the loop
            if (dir_beta == 1 and self.beta > beta_e) or (dir_beta == -1 and self.beta < beta_e):
                last_iteration = True

            d_c = 2 * self.rB * sin(self.beta / 2)  # current distance of beta arm from centre

            if not last_iteration:
                if d_c < min_d:  # calc_xy_arc fails if this is true
                    d_c = min_d  # may be caused by very slight calculation errors
                # get current x/y position
                self.x, self.y = self.calc_xy_arc(m, n, r, d_c, pos_neg, dir_x)
            else:
                # compensate for rounding errors on the last go by simply using the target values
                # as current position; seems to be fine
                self.x = x_e
                self.y = y_e

            if self.DEBUGOUT:  # dev stuff
                self.DEBUGOUT.update_position(self.x, self.y, 'arc')

            steps_alpha, dir_alpha = self.calc_alpha(self.x, self.y)

            # add to path
            self.segment_index += 1
            path_segment = PathSegment(dir_alpha * steps_alpha, dir_beta, 0, self.x, self.y, self.z,
                                       self.alpha, self.beta, self.current_gcommand, self.segment_index)
            path.append(path_segment)

            # checking whether to continue loop or not
            if last_iteration:
                return path

    def calc_xy_arc(self, m, n, r, d_c, pos_neg, dir_x):
        # calculate the two possible x and y positions based on the distance between beta arm and centre
        # decide which set is correct and return x_c and y_c

        # working with circle equation: f(x) = sqrt(r^2 + (x-m)^2) + n
        # d = sqrt(x^2 + f(x)^2)
        # transformed to x = .... you get the following:

        a = 4 * m ** 2 + 4 * n ** 2
        b = -4 * m * (d_c ** 2 - r ** 2 + m ** 2 - n ** 2) - 8 * n ** 2 * m
        c = (d_c ** 2 - r ** 2 + m ** 2 - n ** 2) ** 2 - 4 * n ** 2 * (r ** 2 - m ** 2)

        inner = b ** 2 - 4 * a * c
        if inner < 0:
            # because of slight errors the calculation can give a result marginally smaller than zero (e.g. -10e-9)
            # this results in an error as negative numbers have no square root; this code corrects this issue
            inner = 0

        x_c1 = (-b + sqrt(inner)) / (2 * a)
        x_c2 = (-b - sqrt(inner)) / (2 * a)

        # now we need to decide whether x_c1 or x_c2 is the correct value
        # this is maybe not the best way to do it
        # let's see how it works
        if dir_x == 1:
            if x_c1 > self.x and x_c2 > self.x:
                if abs(x_c2 - self.x) > abs(x_c1 - self.x):
                    x_c = x_c1
                else:
                    x_c = x_c2
            elif x_c1 > self.x:
                x_c = x_c1
            else:
                x_c = x_c2
        else:
            if x_c1 < self.x and x_c2 < self.x:
                if abs(x_c2 - self.x) > abs(x_c1 - self.x):
                    x_c = x_c1
                else:
                    x_c = x_c2
            elif x_c1 < self.x:
                x_c = x_c1
            else:
                x_c = x_c2

        try:
            y_c = pos_neg * sqrt(r ** 2 - (x_c - m) ** 2) + n
        except ValueError:
            y_c = n

        return x_c, y_c

    def calc_alpha(self, x, y):
        # get gamma (angle between y axis and line connecting centre and current position)
        self.gamma = self.calc_gamma(x, y)

        # calculate new alpha value
        delta_c = 0.5 * self.beta * self.DISTORTION_CORRECTION
        # delta is the angle between x axis and current position of beta arm on its arc
        alpha_new = self.limit_alpha(-self.gamma + delta_c)
        # alpha is the angle of the table (range: -pi to +pi)

        # calculate steps/direction to move alpha axis
        alpha_dif, dir_alpha = self.alpha_dif(self.alpha, alpha_new)

        steps_alpha = int(round(alpha_dif / self.ALPHA_RES, 0))

        # recalculate true alpha position (not the same as the before calculate one as it can only move in steps)
        self.alpha = self.limit_alpha(self.alpha + steps_alpha * self.ALPHA_RES * dir_alpha)

        return steps_alpha, dir_alpha

    def calc_gamma(self, x, y):
        # gamma is the angle between x axis and a line connecting centre point and current position
        # depending on the x and y values the calculation slightly differs
        if x == 0.0 and y == 0.0:
            # if both x and y are 0 , the position of the table does not matter
            # just return current value to prevent unnecessary movement
            return self.gamma
        else:
            if (x >= 0) and (y >= 0):
                try:
                    return atan(y / x)
                except ZeroDivisionError:
                    return 0.5 * pi
            elif (x >= 0) and (y < 0):
                try:
                    return atan(y / x)
                except ZeroDivisionError:
                    return -0.5 * pi
            elif (x < 0) and (y < 0):
                try:
                    return -atan(x / y) - 0.5 * pi
                except ZeroDivisionError:
                    return -pi
            elif (x < 0) and (y >= 0):
                if y == 0:
                    return pi
                try:
                    return abs(atan(x / y)) + 0.5 * pi
                except ZeroDivisionError:
                    return pi

    @staticmethod
    def limit_alpha(value):
        if not -pi <= value <= pi:
            value = ((value + pi) % (2 * pi)) - pi

        return value

    @staticmethod
    def alpha_dif(v1, v2):
        dif = v2 - v1
        dir = sign(dif)

        dif = abs(dif)
        if dif > pi:
            dif = (2 * pi) - dif
            dir *= -1

        return dif, dir


class ShouldNotBePossibleError(BaseException):  # TODO improve or remove
    pass
