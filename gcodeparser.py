# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# parser for gcode, creates a gcommand object for each line (see datastructures.py)

from datastructures import GCommand


def loc_key_value(keys, line):
    # loc = location of key
    # end_loc = end of associated value

    # locate keys and create {loc: key,...}
    loc_key = dict()
    for key in keys:
        loc = line.find(key)
        if not loc == -1:
            loc_key[loc] = key

    if loc_key:
        # sort locs ascending and sort keys ascending based on loc
        locs_sorted = sorted(loc_key)
        keys_sorted = list()
        for loc in locs_sorted:
            keys_sorted.append(loc_key[loc])

        # find end of value
        end_locs = list()
        for loc in locs_sorted:
            for i in range(loc+1, len(line)):
                # test whether current char is a number or is a . or -
                try:
                    int(line[i])
                except ValueError:
                    if line[i] == '.' or line[i] == '-':
                        continue
                    else:
                        # end of number reached --> end_loc
                        end_locs.append(i)
                        break

        # create {key: value, ...}
        key_value = dict()
        for i in range(len(locs_sorted)):
            loc = locs_sorted[i]
            if i < len(end_locs):
                key_value[loc_key[loc]] = line[loc+1:end_locs[i]]
            else:  # end_locs might be shorter than locs_sorted if last end_loc equals end of line
                key_value[loc_key[loc]] = line[loc+1:]

        return key_value

    else:
        return {}


class GcodeParser:
    def __init__(self):
        self.index = int(0)

    def file(self, filename):
        # read file and parse gcode
        # returns gcode as (example): [('G01', {'Z': '-1.000000', 'Y': '25.513603 ', 'X': '10.615510 '}), ....]
        with open(filename, "r") as infile:  # read gcode from file
            raw = infile.readlines()
            infile.close()

        gcommand = list()
        for line in raw:
            line = line.replace(' ', '').strip('\n')  # remove all whitespaces and linebreaks

            if line[:3] in ('G00', 'G01'):
                key_value = loc_key_value(('X', 'Y', 'Z', 'F'), line)
                gcommand.append(GCommand(gtype=line[:3], setup=False, gcode=key_value, index=self.index))
                self.index += 1

            elif line[:3] in ('G02', 'G03'):
                key_value = loc_key_value(('X', 'Y', 'Z', 'F', 'I', 'J'), line)
                gcommand.append(GCommand(gtype=line[:3], setup=False, gcode=key_value, index=self.index))
                self.index += 1

        return gcommand

