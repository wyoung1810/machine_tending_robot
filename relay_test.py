from sainsmart_relay.relay import FTDIBitbangRelay
import time

# Initialize the relay controller with the device ID
relay_controller = FTDIBitbangRelay(device_id='AB0NXT9F')

# Turn relay 1 on
# for i in range(4):
#     relay_controller.set_relay(i+1, 'off')
#     time.sleep(1)
relay_controller.set_relay(1, 'off')
    
    

# Retrieve the current relay state
state = relay_controller.get_relay_state()
print(f"Current Relay State: {state:08b}")