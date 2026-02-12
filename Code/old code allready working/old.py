import tkinter as tk
import time
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017
import RPi.GPIO as GPIO

# MCP23017 addresses
MCP23017_ADDR_1 = 0x20  # Solenoids + Thrubeam sensor
MCP23017_ADDR_2 = 0x21  # Reed sensors

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MCP23017 for solenoids and reed sensors
mcp_solenoids = MCP23017(i2c, address=MCP23017_ADDR_1)
mcp_reed_sensors = MCP23017(i2c, address=MCP23017_ADDR_2)

# Solenoid and Reed Sensor State Management
solenoid_states = [0] * 8
reed_sensor_states = [0] * 16
errors = []
stop_auto = False
manual_mode_active = False
motor_speed = 50  # Default motor speed percentage

# Set solenoid pins as outputs
solenoid_pins = [mcp_solenoids.get_pin(i) for i in range(8)]
for pin in solenoid_pins:
    pin.switch_to_output(value=False)

# Set reed sensor pins as inputs with pull-up resistors
reed_sensor_pins = [mcp_reed_sensors.get_pin(i) for i in range(16)]
for pin in reed_sensor_pins:
    pin.switch_to_input(pull=digitalio.Pull.UP)

# Thru-beam sensor setup at MCP23017_ADDR_1 pin 16
thru_sensor_pin = mcp_solenoids.get_pin(15)
thru_sensor_pin.switch_to_input(pull=digitalio.Pull.UP)

# Track sensor state for edge detection
last_sensor_state = thru_sensor_pin.value

# Adjustable parameter from GUI
adjusted_homing_step = 100  # Default value, can be updated via GUI

# Sequence for solenoids
sequence = [
    [0, 0, 0, 0, 0, 0, 0, 0], #Step 1
    [1, 0, 0, 1, 0, 0, 0, 0], #Step 2
    [1, 0, 1, 1, 1, 0, 0, 0], #Step 3
    [1, 0, 0, 1, 1, 1, 0, 1], #Step 4
    [0, 0, 0, 1, 0, 1, 1, 1], #Step 5
    [0, 0, 0, 0, 0, 0, 0, 0], #Step 6
]

def read_reed_sensors():
    global reed_sensor_states
    reed_sensor_states = [not pin.value for pin in reed_sensor_pins]  # Read reed sensors (invert since pull-up is used)
    print(f"Reed sensor states: {reed_sensor_states}")
    return reed_sensor_states
    
def write_solenoids():
    for i, pin in enumerate(solenoid_pins):
        pin.value = solenoid_states[i]
    print(f"Solenoid states: {solenoid_states}")

def verify_state(solenoid_num, state):
    reed_sensors = read_reed_sensors()
    default = reed_sensors[solenoid_num] == 1 and reed_sensors[solenoid_num + 8] == 0
    actuate = reed_sensors[solenoid_num] == 0 and reed_sensors[solenoid_num + 8] == 1
    print(f"Verifying solenoid {solenoid_num}, state: {state}, result: {'Pass' if (default or actuate) else 'Fail'}")
    if state == 'default' and default:
        return True
    elif state == 'actuate' and actuate:
        return True
    else:
        return False

def actuate_solenoid(solenoid_num):
    solenoid_states[solenoid_num] = 1
    write_solenoids()
    time.sleep(0.3)
    if verify_state(solenoid_num, 'actuate'):
        root.after(3000, lambda: reset_solenoid(solenoid_num))  # Use root.after instead of time.sleep for GUI responsivenes
        return True
    else:
        errors.append(f"Solenoid {solenoid_num} failed to actuate.")
        update_gui()
        return False

def reset_solenoid(solenoid_num):
    solenoid_states[solenoid_num] = 0
    write_solenoids()
    time.sleep(0.3)
    time.sleep(0.6)
    if not verify_state(solenoid_num, 'default'):
        errors.append(f"Solenoid {solenoid_num} failed to return to default state.")
    time.sleep(0.2)
    update_gui()
    
# Define step-wise execution in Auto Mode
def execute_sequence_step_with_feedback(step, callback=None):
    if step >= len(sequence):
        if callback:
            callback()
        return

    def validate_step_and_continue():
        solenoid_states[:] = sequence[step]
        write_solenoids()
        update_gui()
        
        # Wait 500ms for solenoids to actuate
        root.after(500, lambda: validate_feedback_for_step(step, callback))

    validate_step_and_continue()

def validate_feedback_for_step(step, callback):
    reed_sensors = read_reed_sensors()
    step_passed = True
    for solenoid_num, state in enumerate(sequence[step]):
        if state == 1:
            actuate = reed_sensors[solenoid_num] == 0 and reed_sensors[solenoid_num + 8] == 1
            if not actuate:
                errors.append(f"[Step {step}] Solenoid {solenoid_num} failed feedback check.")
                step_passed = False

    if step_passed:
        next_step = step + 1
        root.after(300, lambda: execute_sequence_step_with_feedback(next_step, callback))
    else:
        stop_auto_mode()
        update_gui()

def auto_mode():
    global stop_auto, manual_mode_active
    stop_auto = False
    manual_mode_active = False

    def cycle():
        if stop_auto:
            return
        execute_sequence_step_with_feedback(0, run_stepper_motor)

    cycle()

    
def stop_auto_mode():
    global stop_auto
    stop_auto = True
    print("Auto Mode Stopped.")

# Manual Mode Functions for Each Section
def manual_mode():
    global manual_mode_active, stop_auto
    stop_auto = True  # Stop auto mode if it's running
    manual_mode_active = True
    print("Manual Mode Activated.")

def manual_pre_assembly():
    if manual_mode_active:
        actuate_manual_sequence(0, 3)  # Solenoids 1-3 (Pre-assembly)

def manual_main_assembly():
    if manual_mode_active:
        actuate_manual_sequence(3, 6)  # Solenoids 4-6 (Main assembly)
        
def manual_post_assembly():
    if manual_mode_active:
        actuate_manual_sequence(6, 8)  # Solenoids 7-8 (Post-assembly)

# Function to actuate solenoid sequence in manual mode
def actuate_manual_sequence(start, end):
    def actuate_next_manual_step(step):
        if step < len(sequence):
            for i in range(start, end):
                solenoid_states[i] = sequence[step][i]
            write_solenoids()
            update_gui()
            root.after(400, lambda: actuate_next_manual_step(step + 1))  # 2 seconds delay
        else:
            bring_all_to_default()
    actuate_next_manual_step(0)
    
def reset_system():
    global stop_auto, manual_mode_active
    stop_auto = False
    manual_mode_active = False
    errors.clear()
    bring_all_to_default()
    update_gui()

# Bring all solenoids to default state at startup
def bring_all_to_default():
    for solenoid_num in range(8):
        solenoid_states[solenoid_num] = 0
    write_solenoids()
    update_gui()

# Stepper Motor Control with homing sensor check
step_pin = 12  # GPIO pin for STEP
dir_pin = 16  # GPIO pin for DIR
direction = GPIO.LOW  # Default motor direction

GPIO.setmode(GPIO.BCM)
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)

def run_stepper_motor():
    global motor_speed, step_delay
    steps_per_revolution = 3200  # Microstepping: 3200 pulses per revolution
    desired_steps = 984 # 100 steps out of 3200
    step_delay = 1 / (motor_speed * steps_per_revolution / 60)  # Calculate delay per step based on RPM
    global adjusted_homing_step, last_sensor_state
    motor_stopped = False
    step_count = 0


    print(f"Running stepper motor at {motor_speed}% speed for {desired_steps} steps.")

#    for step in range(desired_steps):
    while step_count < desired_steps:
        current_sensor_state = thru_sensor_pin.value
#       print(f"Current state: {current_sensor_state}")
#        print(f"Last sensor state before : {last_sensor_state}")
                # Detect falling edge: 1 -> 0
        if last_sensor_state == 0 and current_sensor_state == 1:
            print("[EVENT] Falling edge detected! Initiating homing adjustment...")
            last_sensor_state = current_sensor_state
            motor_stopped = True
            time.sleep(2)
            for i in range(adjusted_homing_step):
                GPIO.output(step_pin, GPIO.HIGH)
                time.sleep(step_delay)
                GPIO.output(step_pin, GPIO.LOW)
                time.sleep(step_delay * 2)
                print("[INFO] Adjusted homing completed.")
            time.sleep(1)
#            stop_auto_mode()
            break
        last_sensor_state = current_sensor_state
        
        if motor_stopped == False:
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(step_delay / 2)  # Half-step HIGH
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(step_delay / 2)  # Half-step LOW
            step_count += 1
#        print(f"Last sensor state : {last_sensor_state}")
        print(f"step_count : {step_count}")
        

def homing_adjust():
    for i in range(adjusted_homing_step):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(step_delay / 2)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(step_delay / 2)
        print("[INFO] Adjusted homing completed.")
    last_sensor_state = current_sensor_state
        
        
def toggle_motor_direction():
    global direction
    direction = GPIO.HIGH if direction == GPIO.LOW else GPIO.LOW
    GPIO.output(dir_pin, direction)
    time.sleep(0.2)
    print(f"Motor direction: {'Clockwise' if direction == GPIO.HIGH else 'Counter-Clockwise'}")

# Tkinter GUI Setup
root = tk.Tk()
root.geometry("800x480")  # 7-inch display resolution
root.title("PLC-Like Solenoid Control GUI")

# GUI Labels for each solenoid section
pre_labels = []
main_labels = []
post_labels = []

# Pre-assembly section solenoids (1, 2, 3)
tk.Label(root, text="Pre-assembly", font=("Helvetica", 12)).grid(row=0, column=0, pady=5)
for i in range(3):
    lbl = tk.Label(root, text=f"Solenoid {i+1}: Default", width=20, bg="green", anchor="w")
    lbl.grid(row=i+1, column=0, padx=5, pady=5)
    pre_labels.append(lbl)

# Main assembly section solenoids (4, 5, 6)
tk.Label(root, text="Main assembly", font=("Helvetica", 12)).grid(row=0, column=1, pady=5)
for i in range(3):
    lbl = tk.Label(root, text=f"Solenoid {i+4}: Default", width=20, bg="green", anchor="w")
    lbl.grid(row=i+1, column=1, padx=5, pady=5)
    main_labels.append(lbl)

# Post-assembly section solenoids (7, 8)
tk.Label(root, text="Post-assembly", font=("Helvetica", 12)).grid(row=0, column=2, pady=5)
for i in range(2):
    lbl = tk.Label(root, text=f"Solenoid {i+7}: Default", width=20, bg="green", anchor="w")
    lbl.grid(row=i+1, column=2, padx=5, pady=5)
    post_labels.append(lbl)

# Error list
error_listbox = tk.Listbox(root, height=6, width=80)
error_listbox.grid(row=6, columnspan=3, padx=10, pady=5)

# Update GUI to reflect solenoid states and errors
def update_gui():
    for i, lbl in enumerate(pre_labels):
        lbl.config(bg="green" if solenoid_states[i] == 0 else "red")
    for i, lbl in enumerate(main_labels):
        lbl.config(bg="green" if solenoid_states[i+3] == 0 else "red")
    for i, lbl in enumerate(post_labels):
        lbl.config(bg="green" if solenoid_states[i+6] == 0 else "red")
    error_listbox.delete(0, tk.END)
    for error in errors:
        error_listbox.insert(tk.END, error)

# Motor Speed Control
speed_label = tk.Label(root, text=f"Motor Speed: {motor_speed}%", font=("Helvetica", 12))
speed_label.grid(row=12, column=0, padx=5, pady=5)

def set_motor_speed(value):
    global motor_speed
    motor_speed = int(value)
    speed_label.config(text=f"Motor Speed: {motor_speed}%")
    print(f"Motor speed set to {motor_speed}%")  # For debugging or confirmation purposes

# Motor speed slider (scale widget)
speed_scale = tk.Scale(root, from_=10, to=100, orient=tk.HORIZONTAL, command=set_motor_speed, length=300)
speed_scale.set(motor_speed)  # Set the initial value to 50%
speed_scale.grid(row=13, column=0, padx=5, pady=5)

# Buttons for manual control
tk.Button(root, text="Manual Mode", command=manual_mode).grid(row=4, column=0, pady=5)
tk.Button(root, text="Pre-assembly", command=manual_pre_assembly).grid(row=5, column=0, pady=5)
tk.Button(root, text="Main assembly", command=manual_main_assembly).grid(row=5, column=1, pady=5)
tk.Button(root, text="Post-assembly", command=manual_post_assembly).grid(row=5, column=2, pady=5)
tk.Button(root, text="Auto Mode", command=auto_mode).grid(row=4, column=1, pady=5)
tk.Button(root, text="Stop Auto", command=stop_auto_mode).grid(row=4, column=2, pady=5)
tk.Button(root, text="Reset System", command=reset_system).grid(row=4, column=3, pady=5)
tk.Button(root, text="Run Stepper Motor", command=run_stepper_motor).grid(row=5, column=3, pady=5)
tk.Button(root, text="Toggle Motor Direction", command=toggle_motor_direction).grid(row=6, column=3, pady=5)

# Initialize system
bring_all_to_default()

# Start GUI main loop
root.mainloop()