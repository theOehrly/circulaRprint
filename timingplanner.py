# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# SegmentAccelerationPlanner and StepAccelerationPlanner
# The SegmentAP manages the acceleration from segment to segment.
# The StepAP manages the per step acceleration within a segment.
#
# Acknowledgement:
# ... and may somebody else help you back on track when you're lost.
# Thank you to Grbl for inspiration and to the one who wrote the extensive and useful comments.
# for comparison see https://github.com/gnea/grbl/blob/master/grbl/planner.c

# Calculations heavily rely on the fact that every PathSegment contains ONE (!) Beta axis step at max.
# This is guaranteed by the way the XYPlaner handles path calculations.


from math import sqrt, radians, ceil
from datastructures import PathSegment


class SegmentAccelerationPlanner:
    # Calculates the acceleration plan based on the following guidelines:
    #
    #       1.  No junction speed may be higher than the specified junction speed limits or the entry/exit
    #           speeds of neighbouring segments.
    #       2.  The velocity difference between entry and exit speed of one segment has to be within the
    #           allowable acceleration/deceleration limits for the head and all axes.
    #       3.  Every segment should run at the acceleration limit so the speed is as high as possible
    #           within the limits.
    #
    # Therefore the planner goes over every segment first in reverse order always making sure that the deceleration is
    # always at the maximum limit for reaching the entry speed of the following segment.
    # The last segment is always planned from a full stop to ensure that deceleration to a full stop is always possible.
    #
    # After that the planner goes over every segment in chronological (forward) order to ensure that no acceleration is
    # higher than allowed. It dials down the entry speeds of the next segments accordingly.
    #
    # To increase computational efficiency the planner checks during the forward pass up to which index of the buffer
    # the acceleration is already optimally planned. A indicator is set to this index. On the next go the planner only
    # calculates from the back of the buffer up to that index and back.
    #
    # If the buffer length is set too low, it can happen that the maximum speed is never reached as the total distance
    # in the buffer is not long enough to accelerate to the maximum speed.

    def __init__(self, config, provider):
        # load data from configuration
        self.FEED_RATE_MM = float(config['feed_rate'])
        self.ACCELERATION = float(config['acceleration'])
        self.MINIMUM_SPEED = float(config['minimum_feed_rate'])
        self.JUNCTION_DEVIATION = float(config['junction_deviation'])

        self.BETA_RADIUS = float(config['rB'])

        self.ALPHA_RES = float(radians(config['alpha_res']))
        self.BETA_RES = float(radians(config['beta_res']))
        self.ALPHA_ACCELERATION = float(config['a_acceleration'])
        self.BETA_ACCELERATION = float(config['b_acceleration'])

        # load max axis feed rates and convert from mm/sec to degree/sec
        self.ALPHA_FEED_RATE = float(config['a_feed_rate']) * self.ALPHA_RES
        self.BETA_FEED_RATE = float(config['b_feed_rate']) * self.BETA_RES
        self.Z_DELAY = 1 / float(config['z_feed_rate'])

        # calculate maximum beta acceleration in mm/sec^2 from steps/sec^2
        # this is calculated independently per segment for alpha,
        # as the alpha radius changes depending on the current position
        self.BETA_ACCELERATION_MM = self.BETA_ACCELERATION * self.BETA_RADIUS * self.BETA_RES

        self.previous_nominal_speed = 0
        self.previous_segment_x = 0
        self.previous_segment_y = 0
        self.previous_x_unit_vec = 0
        self.previous_y_unit_vec = 0
        self.previous_alpha = 0
        self.previous_beta = 0

        # the processing buffer is used to store the segments while they are used for calculations
        # testing revealed that a list seems to be fastest in the current implementation
        # an experimental ringbuffer was faster shifting the data through,
        # but it had to much overhead for iterations through the whole buffer
        self.processing_buffer = list()
        self.buffer_length = int(config['SegAP_buffer_length'])

        # buffer_planned indicates up to which index the buffer is already calculated optimally
        # After recalculation is done, all segments which are already optimal are deleted from the
        # processing buffer. After that the buffer is topped up again.
        self.buffer_planned = 0

        # completed segments stores segments which are already fully calculated until they are requested
        self.completed_segments = list()

        self.bypass = list()

        self.last_segment_index = int(0)

        self.provider = provider  # class which provides new segments

    def serve_next(self):
        if self.bypass and self.bypass[0].index == self.last_segment_index + 1:
            self.last_segment_index = self.bypass[0].index
            return self.bypass.pop(0)

        if self.completed_segments:
            self.last_segment_index = self.completed_segments[0].index
            return self.completed_segments.pop(0)

        else:
            while len(self.processing_buffer) < self.buffer_length:
                segment = self.provider.serve_next()
                if not segment:
                    break

                segment, go_on = self.initiate_segment(segment)
                if go_on:
                    self.processing_buffer.append(segment)
                else:
                    if segment.index == self.last_segment_index + 1:
                        self.last_segment_index = segment.index
                        return segment
                    else:
                        self.bypass.append(segment)

            if not self.processing_buffer:
                return None

            if len(self.processing_buffer) > 1:
                self.recalculate_segment_buffer()

            if self.buffer_planned < 1:
                self.last_segment_index = self.processing_buffer[0].index
                ret = self.processing_buffer.pop(0)
                return ret
            else:
                self.completed_segments = self.processing_buffer[:self.buffer_planned + 1]
                del self.processing_buffer[:self.buffer_planned + 1]
                self.buffer_planned = 1

                self.last_segment_index = self.completed_segments[0].index
                return self.completed_segments.pop(0)

    def recalculate_segment_buffer(self):
        # reverse pass through the processing buffer
        # always starts with an empty segment at the end to represent a stop

        current_seg = PathSegment(0, 0, 0, 0, 0, 0, 0, 0, None, None)

        for idx in range(-1, -len(self.processing_buffer) + self.buffer_planned, -1):
            next_seg = current_seg
            current_seg = self.processing_buffer[idx]

            # calculate the maximum entry speed by decelerating over the current segment
            # (mathematically we accelerate in reverse)
            entry_speed_sqr = next_seg.entry_speed_sqr + 2 * current_seg.acceleration * current_seg.distance

            if entry_speed_sqr < current_seg.max_entry_speed_sqr:
                current_seg.entry_speed_sqr = entry_speed_sqr
            else:
                current_seg.entry_speed_sqr = current_seg.max_entry_speed_sqr

        # forward_pass
        next_seg = self.processing_buffer[self.buffer_planned]
        for idx in range(self.buffer_planned + 1, len(self.processing_buffer), 1):
            current_seg = next_seg
            next_seg = self.processing_buffer[idx]

            # calculate maximum entry speed by accelerating over the current segment
            if current_seg.entry_speed_sqr < next_seg.entry_speed_sqr:
                entry_speed_sqr = current_seg.entry_speed_sqr + 2 * current_seg.acceleration * current_seg.distance
                if entry_speed_sqr < next_seg.entry_speed_sqr:
                    next_seg.entry_speed_sqr = entry_speed_sqr
                    # the current segment is at maximum acceleration, the buffer can not be improved anymore up to here
                    # the buffer_planned indicator gets set to the new position
                    self.buffer_planned = idx - 2

    def limit_feed_rate_by_axis_limit(self, segment):
        # limits the feed rate for this segment to the lowest value of head, alpha and beta

        # calculate the minimum duration for the segment based on how far the axes
        # need to move and their maximum speeds  (time = distance / velocity)
        alpha_dif = abs(self.previous_alpha - segment.alpha)
        t_min_a = alpha_dif / self.ALPHA_FEED_RATE

        beta_dif = abs(self.previous_beta - segment.beta)
        t_min_b = beta_dif / self.BETA_FEED_RATE

        # calculate maximum feed rate in mm/sec (velocity = distance / time)

        axis_feed_limit_mm = segment.distance / max(t_min_a, t_min_b)

        return min(self.FEED_RATE_MM, axis_feed_limit_mm)

    def limit_acceleration_by_axis_limit(self, segment):
        # limits the acceleration in this segment to the axes' maximum and the head's maximum
        # head acceleration is always relevant so it is added to the list of possible limits
        # by default
        acceleration_values = [self.ACCELERATION]

        # The alpha axis acceleration limit is set in steps/sec^2. This corresponds to a different amount
        # of mm/sec^2 depending on the distance of the current position to the table's center.
        # Therefore the acceleration limit in mm/sec^2 needs to be calculated for every segment
        # if the alpha axis needs to move.
        # In theory it needs to be calculated for every step but per segment should be exact enough.
        if segment.a_steps:
            alpha_dif = abs(self.previous_alpha - segment.alpha)
            distance_to_centre = sqrt(segment.x ** 2 + segment.y ** 2)  # TODO set value in PathPlanner
            distance_per_alpha_step = (distance_to_centre * alpha_dif) / abs(segment.a_steps)

            alpha_acceleration_mm = self.ALPHA_ACCELERATION * distance_per_alpha_step  # mm/s^2 = steps/s^2 * mm/step

            acceleration_values.append(alpha_acceleration_mm)

        # If the beta axis needs to move, the precalculated acceleration limit is added
        # to the list of possible values
        if segment.b_steps:
            acceleration_values.append(self.BETA_ACCELERATION_MM)

        return min(acceleration_values)

    def initiate_segment(self, segment):
        # All necessary values for further calculations are set here.
        # After this the segment can be loaded into the processing buffer.
        #
        # Technically some of these calculations are only suitable for cartesian printers.
        # Though, as one segment covers a very short distance only (normally),
        # everything should be approximately right. (For very short distances the movement is almost linear)

        # set z axis delay
        segment.z_delay = self.Z_DELAY

        # calculate distance covered by segment
        x_distance = segment.x - self.previous_segment_x
        y_distance = segment.y - self.previous_segment_y
        segment.distance = sqrt(x_distance ** 2 + y_distance ** 2)

        if not segment.a_steps and not segment.b_steps:
            return segment, False  # filter out zero-length segments

        # calculate unit vectors
        segment.x_unit_vector = x_distance / segment.distance
        segment.y_unit_vector = y_distance / segment.distance

        # calculate feedrate and acceleration
        # TODO: add G00/G01 differences here
        segment.nominal_speed = self.limit_feed_rate_by_axis_limit(segment)
        segment.acceleration = self.limit_acceleration_by_axis_limit(segment)

        # calculate angle between previous and current path
        # the angle is the dot product of the two vectors
        # the calculation of the dot product is simplified through the usage of the unit vectors
        junction_cos_theta = -self.previous_x_unit_vec * segment.x_unit_vector - self.previous_y_unit_vec * segment.y_unit_vector

        if junction_cos_theta > 0.999999:
            # angle is 0 degrees i.e. the path makes a full turn
            segment.max_junction_speed_sqr = self.MINIMUM_SPEED ** 2

        elif junction_cos_theta < -0.999999:
            # angle is 180 degrees i.e. the junction is a straight line
            segment.max_junction_speed_sqr = float('inf')

        else:
            sin_theta_d2 = sqrt(0.5 * (1.0 - junction_cos_theta))
            # Trig half angle identity. Always positive. (Whatever that means; just taken from Grbl; it works...yeahy ;)

            # TODO: segment.acceleration is better replaced with junction_acceleration (see grbl)
            segment.max_junction_speed_sqr = max((self.MINIMUM_SPEED ** 2,
                                                  (segment.acceleration * self.JUNCTION_DEVIATION * sin_theta_d2) /
                                                  (1 - sin_theta_d2)
                                                  ))
        # calculate max entry speed
        if segment.nominal_speed > self.previous_nominal_speed:
            segment.max_entry_speed_sqr = self.previous_nominal_speed ** 2
        else:
            segment.max_entry_speed_sqr = segment.nominal_speed ** 2
        if segment.max_entry_speed_sqr > segment.max_junction_speed_sqr:
            segment.max_entry_speed_sqr = segment.max_junction_speed_sqr

        # these variables are needed as a reference back to the current segment when calculating the next one
        self.previous_nominal_speed = segment.nominal_speed
        self.previous_x_unit_vec = segment.x_unit_vector
        self.previous_y_unit_vec = segment.y_unit_vector
        self.previous_segment_x = segment.x
        self.previous_segment_y = segment.y
        self.previous_alpha = segment.alpha
        self.previous_beta = segment.beta

        return segment, True


class StepAccelerationPlanner:
    # Calculates the per step timing from segment entry and exit speeds
    # Goals are:
    #       1. Run both axis at as high as possible speeds for as long as possible
    #       2. Stay within all acceleration limits
    #
    # The planner accelerates/decelerates or cruises the axes so that the required exit speeds are reached.
    # This should always be possible within the per axis acceleration limits, as these are taken into account
    # when calculating the per segment acceleration in SegmentAccelerationPlanner.
    # If there are are enough steps to accelerate higher than the exit speed and decelerate again in time,
    # the planner will do so to maximize running speeds.

    def __init__(self, config, provider):
        # Load data from configuration file
        self.ALPHA_MAX_RATE = float(config['a_feed_rate'])
        self.BETA_MAX_RATE = float(config['b_feed_rate'])
        self.ALPHA_ACCELERATION = float(config['a_acceleration'])
        self.BETA_ACCELERATION = float(config['b_acceleration'])
        self.MINIMUM_RATE = float(config['minimum_axis_feed_rate'])

        # current and next segment; the current one is calculated,
        # the next one is needed as reference for the exit speed
        self.current = None
        self.next = None

        self.bypass = list()
        self.last_segment_index = 0

        # current per axis step rates in steps/second
        self.rate_a = float(0)
        self.rate_b = float(0)

        self.provider = provider

    def serve_next(self):
        while 'this loop just goes on until a segment is returned':
            if self.bypass and self.bypass[0].index == self.last_segment_index + 1:
                self.last_segment_index = self.bypass[0].index
                return self.bypass.pop(0)

            while not self.next:
                # should only happen on first call and maybe next few
                # while there has not been a usable segment for self.next yet
                next_segment = self.provider.serve_next()
                if next_segment.a_steps and next_segment.b_steps:
                    self.next = next_segment
                else:
                    self.last_segment_index = next_segment.index
                    return next_segment

            segment = self.provider.serve_next()
            if not segment:
                segment = PathSegment(0, 0, 0, 0, 0, 0, 0, 0, None, None)
            elif not segment.a_steps and not segment.b_steps:
                self.bypass.append(segment)
                continue

            self.current = self.next
            self.next = segment

            current_max_rate_a, current_max_rate_b = self.nominal_rate(self.current)
            next_max_entry_rate_a, next_max_entry_rate_b = self.max_entry_rate(self.next)

            self.accelerate_a(self.current, current_max_rate_a, next_max_entry_rate_a)
            self.accelerate_b(self.current, current_max_rate_b, next_max_entry_rate_b)

            self.last_segment_index = self.current.index
            return self.current

    def max_entry_rate(self, segment):
        # calculate the maximum entry step rate (steps/sec) for each axis in this segment
        # the calculation assumes the whole segment would be run at its entry speed (which is not really the case)
        # but it gives a theoretical segment duration which allows to calculate the entry rate
        if segment.entry_speed_sqr:
            t = segment.distance / sqrt(segment.entry_speed_sqr)
            max_entry_rate_a = abs(segment.a_steps) / t
            max_entry_rate_b = abs(segment.b_steps) / t

            return max(max_entry_rate_a, self.MINIMUM_RATE), max(max_entry_rate_b, self.MINIMUM_RATE)

        # return a low but non-zero rate
        # if the calculation returned 0 in case of a 0 entry speed, the printer would pause forever
        # also if the returned value is too low, the printer will no longer run smooth at slow speeds
        # a high value will not allow the printer to go slow enough and will also result in non-smooth runs
        return self.MINIMUM_RATE, self.MINIMUM_RATE

    def nominal_rate(self, segment):
        # calculate the nominal (i.e. maximum) step rate per axis for this segment
        # the calculation assumes the whole segment is run at its nominal speed. This results in a minimum
        # segment duration which allows to calculate the maximum step rate
        if segment.nominal_speed:
            t = segment.distance / segment.nominal_speed
            max_rate_a = abs(segment.a_steps) / t
            max_rate_b = abs(segment.b_steps) / t

            return max(max_rate_a, self.MINIMUM_RATE), max(max_rate_b, self.MINIMUM_RATE)

        # return a low but non-zero rate
        # same reasoning as for max_entry_rate() above
        return self.MINIMUM_RATE, self.MINIMUM_RATE

    def accelerate_a(self, segment, max_rate_a, exit_rate_a):
        # this function handles acceleration, cruising and deceleration for the alpha axis within one segment
        # possible scenarios are:
        #                                                    acc
        #  acc        acc              cruise               cruise          cruise      decel
        #  only      cruise             only                decel           decel        only
        #   /         ----         --------------         ----------         ----          \
        #  /         /                                   /          \            \          \
        # /         /                                   /            \            \          \
        #
        # for acc/cruise/decel the entry speed can be lower/equal/higher than the exit speed
        #

        # reverse counting the number of steps gives us the remaining number of steps for this segment
        # i.e. n is the number of steps remaining in this segment
        for remaining in range(abs(segment.a_steps), 0, -1):
            v_dif_a = self.rate_a - exit_rate_a

            # if slower than exit speed, always accelerate
            if v_dif_a < 0:
                self.rate_a += sqrt(2 * self.ALPHA_ACCELERATION)
                # check that neither current maximum nor next maximum entry speed are exceeded
                if self.rate_a > max_rate_a:
                    self.rate_a = max_rate_a
                if self.rate_a > exit_rate_a and remaining == 1:
                    self.rate_a = exit_rate_a

            # if faster than or equal to exit speed
            else:
                # calculate the number of steps it takes to decelerate from current rate to exit_rate
                breaking_dist = ceil((v_dif_a ** 2) / (2 * self.ALPHA_ACCELERATION))

                # accelerate if there are enough steps left for decelerating within the acceleration limit
                # and current step rate is below max rate
                # check against breaking_dist + 1 is necessary because one step of acceleration also requires
                # one more step of deceleration than was required before accelerating
                if breaking_dist + 1 < remaining and self.rate_a < max_rate_a:
                    self.rate_a += sqrt(2 * self.ALPHA_ACCELERATION)
                    # check that current maximum rate is not exceeded
                    if self.rate_a > max_rate_a:
                        self.rate_a = max_rate_a

                # decelerate now
                elif breaking_dist + 1 >= remaining:
                    self.rate_a -= sqrt(2 * self.ALPHA_ACCELERATION)
                    # don't decelerate to a slower speed than required
                    if self.rate_a < exit_rate_a:
                        self.rate_a = exit_rate_a

            # if none of conditions above is met self.rate_a is not change (i.e. cruise)
            # TODO not possible at all?!?
            segment.a_timing.append(1 / self.rate_a)

    def accelerate_b(self, segment, max_rate_b, exit_rate_b):
        # Handles acceleration/cruising/deceleration for the beta axis
        # As each path segment may contain only one beta step, only one of these is possible per segment

        # do nothing if there is no b step
        if not segment.b_steps:
            return

        v_dif_b = self.rate_b - exit_rate_b

        # accelerate, current speed is slower than exit speed
        if v_dif_b < 0:
            self.rate_b += sqrt(2 * self.BETA_ACCELERATION)
            # check that neither current maximum nor next maximum entry speed are exceeded
            if self.rate_b > max_rate_b:
                self.rate_b = max_rate_b
            if self.rate_b > exit_rate_b:
                self.rate_b = exit_rate_b

        # decelerate, faster than exit speed
        elif v_dif_b > 0:
            self.rate_b -= sqrt(2 * self.BETA_ACCELERATION)
            # don't decelerate to a slower speed than required
            if self.rate_b < exit_rate_b:
                self.rate_b = exit_rate_b

        # if none of the above two conditions is met, the axis is already running at the desired speed
        b_delay = 1 / self.rate_b
        segment.b_timing.append(b_delay)
