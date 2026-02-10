# solenoid.py
from io_mcp import solenoid_pins, reed_pins
import state

def write_solenoids():
    for i, p in enumerate(solenoid_pins):
        p.value = state.solenoid_states[i]

def read_reeds():
    state.reed_sensor_states = [not p.value for p in reed_pins]
    return state.reed_sensor_states

def verify(sol, expected):
    r = read_reeds()
    default = r[sol] == 1 and r[sol+8] == 0
    actuate = r[sol] == 0 and r[sol+8] == 1

    if expected == "default":
        return default
    return actuate
