# sequence.py
import state
from solenoid import write_solenoids, verify

sequence = [
    [0,0,0,0,0,0,0,0],
    [1,0,0,1,0,0,0,0],
    [1,0,1,1,1,0,0,0],
    [1,0,0,1,1,1,0,1],
    [0,0,0,1,0,1,1,1],
    [0,0,0,0,0,0,0,0],
]

def execute_step(step):
    state.solenoid_states[:] = sequence[step]
    write_solenoids()
