# Import the smbus2 library
from smbus2 import SMBus
import time

# I2C bus number (1 for Raspberry Pi and most modern boards)
I2C_BUS = 0

# I2C device address (change to your device's address)
DEVICE_ADDRESS = 0x30  

# Register to read from (change based on your device datasheet)
REGISTER_ADDRESS = 0x00  

def read_i2c_device():
    # Open the I2C bus
    with SMBus(I2C_BUS) as bus:
        try:
            # Read a single byte from the register
            data = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_ADDRESS)
            print(f"Data read from device: {data}")
        except Exception as e:
            print(f"Error reading from I2C device: {e}")

if __name__ == "__main__":
    while True:
        read_i2c_device()
        time.sleep(.1)
        
