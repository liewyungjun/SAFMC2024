import time
import cflib.crtp  # Import the Crazyflie API
from cflib.crazyflie import Crazyflie
import keyboard  # Import the keyboard module

# Initialize the Crazyflie API
cflib.crtp.init_drivers()

# Define the URIs of the two Crazyflies
uri1 = 'radio://0/80/2M/E7E7E7E701'
uri2 = 'radio://0/80/2M/E7E7E7E702'

# Connect to the first Crazyflie
cf1 = Crazyflie()
cf1.open_link(uri1)

# Connect to the second Crazyflie
cf2 = Crazyflie()
cf2.open_link(uri2)

# Takeoff and hover with cf1
cf1.commander.send_setpoint(0, 0, 0, 50000)
time.sleep(5)
cf1.commander.send_setpoint(0, 0, 0, 0)

# Set initial setpoints for cf2
roll = 0
pitch = 0
yawrate = 0
thrust = 0

#q,e for yawrate and up,down for thrust

# Define setpoint function for cf2
def setpoint_callback():
    global roll, pitch, yawrate, thrust

    # Read keyboard input
    if keyboard.is_pressed('w'):
        pitch = 20000
    elif keyboard.is_pressed('s'):
        pitch = -20000
    else:
        pitch = 0

    if keyboard.is_pressed('a'):
        roll = -20000
    elif keyboard.is_pressed('d'):
        roll = 20000
    else:
        roll = 0

    if keyboard.is_pressed('q'):
        yawrate = -20000
    elif keyboard.is_pressed('e'):
        yawrate = 20000
    else:
        yawrate = 0

    if keyboard.is_pressed('up'):
        thrust = 50000
    elif keyboard.is_pressed('down'):
        thrust = -50000
    else:
        thrust = 0

    # Send setpoints to cf2
    cf2.commander.send_setpoint(roll, pitch, yawrate, thrust)

# Subscribe to range sensor data from cf1
cf2_range_sub = cf2.subscriber("range.front", "uint16_t")

# Set the setpoint callback function for cf2
cf2.setpoint_sender.set(setpoint_callback)

# Main loop
while True:
    # Read range sensor data from cf1
    range_data = cf2_range_sub.get_last_message()
    if range_data is not None:
        print("Range: {} mm".format(range_data))

    # Sleep for a short time to avoid busy waiting
    time.sleep(0.01)