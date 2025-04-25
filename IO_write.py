import time
from pyfirmata2 import Arduino, OUTPUT, INPUT

# Connect to the Arduino board
board = Arduino("COM4")  # use '/dev/ttyACM0' for Linux or 'COM4' for Windows
print(f"Connected to {board.name}")

# Define the digital pin (e.g., D13)
digital_pin = board.digital[2]

# Set the pin mode to OUTPUT
digital_pin.mode = OUTPUT

# Give the board a moment to initialize
time.sleep(1)

digital_pin.write(False)  # Turn ON

digital_pin.mode = INPUT
digital_pin.enable_reporting()

print(f"Digital: {digital_pin.value}")

# # Blink the LED connected to D13
# try:
#     while True:
#         digital_pin.write(1)  # Turn ON
#         time.sleep(1)
#         digital_pin.write(0)  # Turn OFF
#         time.sleep(1)

# except KeyboardInterrupt:
#     print("Exiting...")
#     board.exit()
