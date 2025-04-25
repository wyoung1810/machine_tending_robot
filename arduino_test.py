from pyfirmata2 import Arduino, util, INPUT
import time

# Connect to the Arduino board
board = Arduino("COM4")  # use '/dev/ttyACM0' for Linux or 'COM4' for Windows
print(f"Connected to {board.name}")

# Start an iterator thread to keep data flowing
it = util.Iterator(board)
it.start()

# Define the pins
digital_pins = [board.digital[2], board.digital[3]]  # D2, D3

# Set digital pins as inputs (assuming reading from them)
for d_pin in digital_pins:
    d_pin.mode = INPUT
    d_pin.enable_reporting()

# Give the board a moment to initialize
time.sleep(1)

# Read loop
try:
    while True:
        # Digital reads
        digital_values = [pin.value for pin in digital_pins]

        print(f"Digital: {digital_values}")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Exiting...")
    board.exit()
