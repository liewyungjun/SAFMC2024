import sys
import tty
import termios
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils.multiranger import Multiranger
import time

# Define the URI of the Crazyflie to connect to
URI = 'radio://0/120/2M/E7E7E7E715'
URI2 = 'radio://0/120/2M/E7E7E7E713'

# Define the velocity of the drone
VELOCITY = 0.1
YAW_RATE = 30 # deg/s

# Define the keyboard input settings
orig_settings = termios.tcgetattr(sys.stdin)

# Define a function to get the keyboard input
def get_input():
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return ch

# Define a function to print the multiranger values
def print_multiranger(multiranger):
    front_range = handle_range_measurement(multiranger.front)
    back_range = handle_range_measurement(multiranger.back)
    left_range = handle_range_measurement(multiranger.left)
    right_range = handle_range_measurement(multiranger.right)
    print('Front: {0:.2f}m, Back: {1:.2f}m, Left: {2:.2f}m, Right: {3:.2f}m'.format(
        front_range, back_range, left_range, right_range))

def handle_range_measurement(range):
    if range is None:
        range = 9999
    return range
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Create a Crazyflie object and connect to the drone
    cf = Crazyflie(rw_cache='./cache')
    cf2 = Crazyflie(rw_cache='./cache2')
    with SyncCrazyflie(URI, cf=cf) as scf,  SyncCrazyflie(URI2, cf=cf2) as scf2:
    #with SyncCrazyflie(URI, cf=cf) as scf:
        # Create a MotionCommander object for controlling the drone
        with MotionCommander(scf) as motion_commander:
        # Create a MotionCommander object for controlling the drone
                    # Create a Multiranger object for reading the multiranger values
                    with Multiranger(scf) as multiranger:
                        # Set the drone to hover
                        #motion_commander.take_off()
                        #motion_commander.hover()
                        motion_commander2 = MotionCommander(scf2)
                        motion_commander2.take_off()
                        print("2 is hovering")
                        print("now start keyboard inputs")

                        # Loop until the user presses 'q'
                        while True:
                            # Get the keyboard input
                            motion_commander2.stop()
                            ch = get_input()
                            motion_commander.forward(0.05,VELOCITY)
                            if ch == 'q':
                                break
                            time.sleep(0.2)

                            # Print the multiranger values
                            print_multiranger(multiranger)

                        # Land the drone and disconnect
                        motion_commander2.land()
                        print("landed and disconnected")