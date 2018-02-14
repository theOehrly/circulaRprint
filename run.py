from rprint import Controller
import time


print('initiating...')
controller = Controller(mode=0)

print('loading file')
controller.load_gcode_file('testfiles/example.ngc')

print('starting...')

t_start = time.time()
controller.run()

controller.move_home()

controller.shutdown()

t_end = time.time()

print('shutdown complete')
print('finished in', round(t_end-t_start, 1), 'seconds')
