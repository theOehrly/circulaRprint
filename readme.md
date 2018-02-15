# **circulaRprint**

**What is circularprint?**  
CirculaRprint is a controller software for circular plotters.
The plotter hardware consists of a rotating table and of an arm that moves across it in and arc. 
This eliminates the need for linear bearings/rods on the two main axis.
The tool tip (i.e. a pen, laser, whatever...) needs to be exactly centered above the table or else there will be a 
dead spot in the middle of the table that can't be reached.
A disadvantage of this approach is the inconsistent resolution, as it gets lower the further you get away from the 
table center (only applies to the table axis, not the arm).

Here are some images that should give you a basic understanding of how the plotter works.  
https://drive.google.com/open?id=1_3LzlJe7sTjrp4OGNJY0WJFvSQl1Wn2D 
    
**Hardware**  
A Raspberry Pi is used as a controller board.  
Also a few H-Bridge motor drivers are needed.  
Of course some stepper motors (I took mine from old DVD drives and printers) and a few more components to build the rest from.

**Features**
* Read Gcode from File (obviously)
* Linear Movement G00/G01 (both treated the same)
* Circular Movement G02/G03
* Individual Axis acceleration and speed control (inspired by GRBL; thank you)
* Backlash correction

**FYI**
The table axis is also called Alpha or A Axis  
The arm is also referred to as beta arm, Beta axis or B Axis

## Configuration
The configuration is a simple python dictionary. Every line needs to end with a comma
(Except for comments behind it). The code mostly does not check whether the configuration is valid,
so if you enter invalid values it'll either crash or do something funny.

#### Explanation of the configuration:

**Key** | **Values** | **Explanation**
--------|------------|----------------
MA, MB, MZ | `[1, 2, 3, 4]` | GPIO pins for alpha, beta and z-axis stepper
M*_full_step_seq | `0` or `1` | Use full step sequence for more torque; results in half the resolution
M*_dir | `1` or `-1` | Changes the direction of the motor; same could be achieved by changing the pin order but this is easier
distortion_correction | `1` or `-1` | Changes whether an offset is added or subtracted during path calculation. For some combinations of motor directions this is necessary. It can't be corrected automatically as it also depends on how the motor is wired up. If straight lines are curved you should try changing this value.
M*_backlash_correction | `[steps, delay]` | Compensates for backlash when the motor/axis changes direction. The first number specifies how many extra steps should be taken. The second number specifies the delay per step. The delay should be approximately (1 divided by axis feed rate). A too low number might result in skipped steps. A too high number might cause non-smooth runs as this extra time is not taken into account when calculating the timing.    
M*_always_unhold | `False` or `True` | If set to True the motor coils are turned off immediately after each movement. This prevents the coils from heating up to much while the motor is holding position. The end position may not be reached correctly though and the motor has no active holding force. Only recommended for slow moving axes that don't move often aka the z-axis. 
rB | `length in mm` | The length of the beta arm in **millimetres** from its point of rotation to the tool tip. Should be measured as accurate as possible. If the tool is offset to the side of the arm, measure from tool tip to pivot point!
alpha_ /beta_res | `resolution in degree/step` | The resolution of the alpha and beta axes. Has to be given in **degree per step**! 
feed_rate | `speed in mm/s` | The nominal speed ot the tool relative to the table
acceleration | `acceleration in mm/s²` | Maximum acceleration of the tool relative to the table
minimum_feed_rate | `speed in mm/s` | Slowest speed for the tool to move at (relative to the table)
junction_deviation | `default: 0.01` | The higher this number is, the faster cornering speeds are. Start out slow!
a_/b_feed_rate | `speed in steps/s` | Maximum absolute speed for this axis.
a_/b_acceleration | `accleration in steps/s² ` | Maximum absolute acceleration for this axis 
z_feed_rate | `speed in mm/s` | Z-axis feed rate; no acceleration is available for z-axis
minimum_axis_feed_rate | `speed in steps/s` | Minimum speed for both alpha and beta axis
SegAP_buffer_length | `number of segments` | How many segments should be used for one calculation run of the SegmentAccelerationPlanner. If the number is too low, maximum speed is never reached. But especially for threaded calculation it should not be too high or it'll take too much time per calculation while doing less calculations overall. Changing this number won't change much about the overall time needed for calculation.
Default_mode | `0` or `1` | Mode 0 calculates all path and acceleration data before starting; this can easily take a minute for larger files. Mode 1 does the calculations in a separate thread while running; the higher CPU load might cause worse timing.  

The acceleration planner always ensures that no speed or acceleration limit is exceeded.  
This means in turn that not every axis is always running at the specified maximum as that isn't possible.


## How to run it
```python
from rprint import Controller

controller = Controller(mode=0, config={})

controller.load_gcode_file('example.ngc')

controller.run()

controller.move_home()
controller.shutdown()
```
This is the basic script for running the circulaRprint controller. It is basically the same as in run.py.  
`mode` and `config` (dict) can be provided optionally, else the defaults (config.py) are used.

I recommend running move_home(). It should not be necessary if the gcode ends at x0 y0, but just to be sure...  
shutdown() finally turns everything off


## Calibration 
#### Resolution
To determine the axis resolution I recommend you to create a python file with the following code:
```python
from rprint import Controller

controller = Controller()

controller.MB.move(100, 0.01)
controller.MA.move(100, 0.01)
controller.MB.move(-100, 0.01)
controller.MA.move(-100, 0.01)

controller.shutdown()
```
`move(steps, delay per step)`  

This should draw a triangle with curved sides. Two sides are drawn by the beta axis and one by the alpha axis.
Increase the number of steps moved in the code above, until the triangle is as big as possible. This makes measuring easier.
Now you need to measure the distance from one corner to the next one.
The following formula helps you to calculate how many degrees the axis moved:

a = arcsin(s /(2*r)) * 2

a = angle  
s = measured distance  

r is the radius of the circle
* for the alpha axis use the distance of the side drawn by it's movement to the centre 
* for the beta axis use the length of the arm as radius r

When you've calculated the angle for each axis, divide it by the number of steps you set in the code. 
Now you have the resolution in degree per step.    

#### Centering
Getting the pen tip centered on the table can be quite tricky. The only thing I can recommend is to lower the pen onto the table.
Then move the table half a rotation by hand. If you can only see a dot, everything is perfect.
If you can see half a circle, the tip is not centered. From the orientation of the circle and the direction you turned the table in, you can conclude the direction you're off in.
Move the position of the tip a bit and try again until no circle is drawn anymore.

## GCode
I only tested gcode produced by the GCode-Tools extension for Inkscape, different gcode might work.
There are a few things one should know about:
1. The controller expects the Z-axis to be at Z=0 when starting (a pen tip would need to be touching the table)
1. A circle segment may cover 180° of a circle at max
1. G00 and G01 are treated exactly the same (the speed my machine was working at was already the maximum my tiny steppers could handle. So I didn't bother to implement a difference.)
1. Feed rates specified in GCode are ignored
1. The code only executes changes of the z-height if there is no x and y information given in the same line (i.e. Z would be ignored in the following command: G00 X5 Y6 Z3 )

GCode should be in millimetres.

It ***might*** work in inch or different units if:
* the length of the arm is specified in inch instead mm
* speeds are given in in inch/s instead mm/s
* acceleration is given in inch/s² instead mm/s²

If it works please contact me so I can update this.

## DebugOut Tool

This is really only intended for debugging. It might crash your system if you draw large files. (It'll remind you)  
! Requires kivy installed ! 
  
To run it, you just start debugout.py  
It will then import the controller and pass an extended version of the default configuration to it.

Configuration of debugout is done directly in debugout.py.  

DebugOut can only be used to verify that the PathPlanner class is working correctly. If anything after that messes up the movement, you won't see it here.
