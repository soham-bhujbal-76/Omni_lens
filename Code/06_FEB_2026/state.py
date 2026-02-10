# state.py

solenoid_states = [0] * 8
reed_sensor_states = [0] * 16

errors = []

stop_auto = False
manual_mode = False

motor_speed = 50
last_thru_state = 1
