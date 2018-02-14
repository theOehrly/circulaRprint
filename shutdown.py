# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# this file exists just for convenience reasons
# if anything goes wrong you can just do sudo python3 shutdown.py
# this will turn of the motors and stop them from overheating as they might not be
# turned of properly depending on where the code crashes (if it crashes)

from rprint import Controller

Controller().shutdown()
