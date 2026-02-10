# io_mcp.py
import board, busio, digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017
from config import *

i2c = busio.I2C(board.SCL, board.SDA)

mcp_sol = MCP23017(i2c, address=MCP23017_ADDR_SOL)
mcp_reed = MCP23017(i2c, address=MCP23017_ADDR_REED)

solenoid_pins = [mcp_sol.get_pin(i) for i in range(NUM_SOLENOIDS)]
reed_pins = [mcp_reed.get_pin(i) for i in range(NUM_REED_SENSORS)]

for p in solenoid_pins:
    p.switch_to_output(False)

for p in reed_pins:
    p.switch_to_input(pull=digitalio.Pull.UP)

thru_sensor = mcp_sol.get_pin(15)
thru_sensor.switch_to_input(pull=digitalio.Pull.UP)
