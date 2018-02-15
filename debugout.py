# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# draws the calculated x-y-points on screen
# as it is drawing based on x-y coordinates if stepper class or the calculation
# of arm and table angle is buggy, you won't see that here!!
#
# REQUIRES KIVY
#
# to run with debugout you need to start debugout.py
# It'll handle importing the controller and modifying the configuration
# you can drag to move the drawing or right-click and then drag with a left-click to zoom in or out

disclaimer = '''
                                 !!!!!! DISCLAIMER !!!!!!
    THIS IS INTENDED AS A DEBUGGING TOOL (ONLY) FOR VERIFYING THE ACCURACY OF A FEW LINES OF G-CODE!
                        Concentric circles are drawn incorrectly!
                    Kivy is used in a way that is probably not intended.
If you'r visualizing long paths or have a high alpha/beta_res set, kivy needs to draw a huge number of points.
          RAM USAGE WILL BECOME UNACCEPTABLY HIGH AND YOU'RE SYSTEM MIGHT BECOME UNRESPONSIVE.
                                Also drawing might take forever.
          When this happens, Python seems to be well capable of CRASHING YOUR SYSTEM.
                I AM NOT RESPONSIBLE FOR ANY ISSUES THAT MIGHT RESULT FROM THAT.
                
                        I can assure you the above out of experience.
                
'''
imadeviunderstand = False

if not imadeviunderstand:
    print(disclaimer)
    input('Press any key to continue')

# ## Config for debugout
# the file which is to be loaded
infile = 'testfiles/example.ngc'

# delay after drawing a point; allows you to watch progress of drawing
delay = 0.00

# draws the gcode directly using native kivy functions
# be aware that this will always draw full circles even if the gcode only specifies a segment of the circle
# it will create a big mess if you draw many lines of gcode
showgcode = False

# draw a point for every calculated xy position
# by default it only draws lines from point to point
# red for linear movement, green for circular movement
showpoints = True

# table radius; used to draw a circle and also necessary for scaling
table_radius = 70

# sets width and height of the window; these values are also used for scaling
width = 700
height = 700
# ## End of config

import threading
import time
from math import sqrt

import kivy

from kivy.app import App
from kivy.config import Config
from kivy.clock import Clock

from kivy.uix.scatter import Scatter

from kivy.graphics.vertex_instructions import Line, Point
from kivy.graphics import Color

from config import config as rprint_config
from rprint import Controller


kivy.require('1.9.1')
Config.set('graphics', 'width', width)
Config.set('graphics', 'height', height)
Config.set('graphics', 'left', 65)
Config.set('graphics', 'top', 35)
Config.set('graphics', 'width', width)
Config.set('graphics', 'position', 'custom')


class Window(Scatter):
    def __init__(self):
        super(Window, self).__init__()
        self.do_rotation = False
        self.currentline = None

        Clock.schedule_once(self.finish_init, 1)

    def finish_init(self, *args):
        debugout = DebugOut(self, rprint_config)
        config = rprint_config
        config['debugout'] = debugout
        controller = Controller(config=config, mode=0)
        controller.load_gcode_file(infile)
        thread = threading.Thread(target=controller.run, args=())
        thread.start()

    def draw_line(self, points, color=(1, 1, 1, 1)):
        if self.currentline:
            self.currentline.points += points
        else:
            with self.canvas:
                Color(rgba=color)
                Line(points=points)

    def draw_circle(self, center_x, center_y, radius, color=(1, 1, 1, 1), angle_start=0, angle_end=360):
        with self.canvas:
            Color(rgba=color)
            Line(circle=(center_x, center_y, radius, angle_start, angle_end))

    def draw_point(self, x, y, color=(1, 1, 1, 1)):
        with self.canvas:
            Color(rgba=color)
            Point(points=[x, y])


class DebugOutputApp(App):
    def build(self):
        return Window()


class DebugOut:
    def __init__(self, ui, config):
        self.ui = ui

        self.r = table_radius

        if width < height:
            self.multiplier = (width / self.r) / 2
        else:
            self.multiplier = (height / self.r) / 2

        self.x = self.mod_coord(0)
        self.y = self.mod_coord(0)
        self.gx = self.mod_coord(0)
        self.gy = self.mod_coord(0)
        self.status = 0

        self.lines = list()
        self.current_line = list()

        # set up ui background
        self.ui.draw_line([0, 0.5 * height, width, 0.5 * height], color=(1, 1, 1, 0.3))
        self.ui.draw_line([0.5 * width, 0, 0.5 * width, height], color=(1, 1, 1, 0.3))
        self.ui.draw_circle(0.5 * width, 0.5 * height, self.r * self.multiplier, color=(1, 1, 1, 0.4))

    def mod_coord(self, coord):
        return int((coord + self.r) * self.multiplier)

    def mod_coord_list(self, coordlist):
        mod = list()
        for coord in coordlist:
            mod.append(self.mod_coord(coord))
        return mod

    def gcode(self, gcommands, x_start=0, y_start=0):
        if not showgcode:
            return

        self.gx = self.mod_coord(x_start)
        self.gy = self.mod_coord(y_start)

        for gcommand in gcommands:
            if gcommand.gtype in ('G00', 'G01'):
                if 'X' in gcommand.gcode:
                    endx = self.mod_coord(float(gcommand.gcode['X']))
                    endy = self.mod_coord(float(gcommand.gcode['Y']))
                    self.ui.draw_line([self.gx, self.gy, endx, endy], color=(0, 1, 1, 1))

                    self.gx = endx
                    self.gy = endy

            elif gcommand.gtype in ('G02', 'G03'):
                centerx = self.gx + float(gcommand.gcode['I']) * self.multiplier
                centery = self.gy + float(gcommand.gcode['J']) * self.multiplier
                endx = self.mod_coord(float(gcommand.gcode['X']))
                endy = self.mod_coord(float(gcommand.gcode['Y']))

                radius = sqrt((centerx - endx) ** 2 + (centery - endy) ** 2)

                self.ui.draw_circle(centerx, centery, radius, color=(0, 1, 1, 1))

                self.gx = endx
                self.gy = endy

    def gcommand_line(self, endx, endy):
        pass

    def gcommand_arc(self):
        pass

    def update_position(self, x, y, mv_type):
        time.sleep(delay)
        x = self.mod_coord(x)
        y = self.mod_coord(y)

        if not self.current_line:
            self.current_line = [self.x, self.y]
        else:
            self.ui.draw_line([self.x, self.y, x, y])
            self.current_line += [x, y]

        if showpoints:
            if mv_type == 'linear':
                self.ui.draw_point(x, y, color=(1, 0, 0, 0.8))
            else:
                self.ui.draw_point(x, y, color=(0, 1, 0, 0.8))

        self.x = x
        self.y = y

    def update_status(self, status):
        if status != self.status:
            self.lines.append(self.current_line)
            self.current_line = list()

            self.status = status


if __name__ == '__main__':
    DebugOutputApp().run()
