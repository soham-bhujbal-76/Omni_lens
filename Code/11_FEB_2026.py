import tkinter as tk
from tkinter import scrolledtext
import time
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017
import RPi.GPIO as GPIO
import logging
from datetime import datetime

# ========== LOGGING CONFIGURATION ==========
# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# Create logger
logger = logging.getLogger(__name__)

# File handler for logging to file
file_handler = logging.FileHandler('machine_log.txt')
file_handler.setLevel(logging.DEBUG)
file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

# ========== CONSTANTS ==========
# MCP23017 addresses
MCP23017_ADDR_1 = 0x20  # Solenoids + Thrubeam sensor
MCP23017_ADDR_2 = 0x21  # Reed sensors

# Timing constants (in seconds)
SOLENOID_ACTUATION_TIME = 0.3
SOLENOID_HOLD_TIME = 3.0
SOLENOID_RESET_TIME = 0.3
SOLENOID_SETTLE_TIME = 0.6
VERIFICATION_DELAY = 0.2
SENSOR_VERIFICATION_TIMEOUT = 2.0
STEP_VALIDATION_DELAY = 0.5
NEXT_STEP_DELAY = 0.3
MOTOR_DIRECTION_SETTLE = 0.2
POST_MOTOR_DELAY = 1.0
HOMING_SETTLE_TIME = 2.0
CYCLE_RESTART_DELAY = 2.0  # Delay before restarting next cycle

# Motor constants
STEPS_PER_REVOLUTION = 3200  # Microstepping: 3200 pulses per revolution
STEPS_PER_CYCLE = 984  # Steps to move per cycle
DEFAULT_HOMING_ADJUSTMENT = 100  # Default homing adjustment steps
DEFAULT_MOTOR_SPEED = 50  # Default motor speed percentage

# GPIO pins for stepper motor
STEP_PIN = 12
DIR_PIN = 16

# ========== HARDWARE INITIALIZATION ==========
# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MCP23017 for solenoids and reed sensors
mcp_solenoids = MCP23017(i2c, address=MCP23017_ADDR_1)
mcp_reed_sensors = MCP23017(i2c, address=MCP23017_ADDR_2)

# GPIO setup for stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# ========== GLOBAL STATE VARIABLES ==========
solenoid_states = [0] * 8
reed_sensor_states = [0] * 16
errors = []
stop_auto = False
manual_mode_active = False
motor_running = False
auto_cycle_count = 0  # Track number of auto cycles completed
motor_speed = DEFAULT_MOTOR_SPEED
adjusted_homing_step = DEFAULT_HOMING_ADJUSTMENT
direction = GPIO.LOW
last_sensor_state = None  # Will be initialized after sensor setup

# ========== SOLENOID PINS SETUP ==========
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

# Initialize motor direction
GPIO.output(DIR_PIN, direction)

# Initialize last sensor state after pin setup
last_sensor_state = thru_sensor_pin.value

# ========== SEQUENCE DEFINITION ==========
sequence = [
    [0, 0, 0, 0, 0, 0, 0, 0],  # Step 1
    [1, 0, 0, 1, 0, 0, 0, 0],  # Step 2
    [1, 0, 1, 1, 1, 0, 0, 0],  # Step 3
    [1, 0, 0, 1, 1, 1, 0, 1],  # Step 4
    [0, 0, 0, 1, 0, 1, 1, 1],  # Step 5
    [0, 0, 0, 0, 0, 0, 0, 0],  # Step 6
]

# ========== GUI LOGGING FUNCTION ==========
def log_to_gui(message, level="INFO"):
    """
    Log message to GUI text widget and system logger
    level: INFO, WARNING, ERROR, DEBUG
    """
    timestamp = datetime.now().strftime('%H:%M:%S')
    formatted_message = f"[{timestamp}] {level}: {message}"
    
    # Log to system logger
    if level == "INFO":
        logger.info(message)
    elif level == "WARNING":
        logger.warning(message)
    elif level == "ERROR":
        logger.error(message)
    elif level == "DEBUG":
        logger.debug(message)
    
    # Log to GUI
    if 'log_text' in globals():
        log_text.insert(tk.END, formatted_message + "\n")
        log_text.see(tk.END)  # Auto-scroll to bottom
        
        # Color coding based on level
        if level == "ERROR":
            log_text.tag_add("error", "end-2l", "end-1l")
            log_text.tag_config("error", foreground="red")
        elif level == "WARNING":
            log_text.tag_add("warning", "end-2l", "end-1l")
            log_text.tag_config("warning", foreground="orange")
        elif level == "DEBUG":
            log_text.tag_add("debug", "end-2l", "end-1l")
            log_text.tag_config("debug", foreground="gray")

# ========== SENSOR & SOLENOID FUNCTIONS ==========
def read_reed_sensors():
    """Read all reed sensor states (inverted for pull-up logic)"""
    global reed_sensor_states
    reed_sensor_states = [not pin.value for pin in reed_sensor_pins]
    log_to_gui(f"Reed sensor states: {reed_sensor_states}", "DEBUG")
    return reed_sensor_states

def write_solenoids():
    """Write current solenoid states to hardware"""
    for i, pin in enumerate(solenoid_pins):
        pin.value = solenoid_states[i]
    log_to_gui(f"Solenoid states: {solenoid_states}", "DEBUG")

def verify_state(solenoid_num, state, timeout=SENSOR_VERIFICATION_TIMEOUT):
    """
    Verify solenoid position using reed sensors with timeout
    Returns True if position verified, False otherwise
    """
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        reed_sensors = read_reed_sensors()
        default = reed_sensors[solenoid_num] == 1 and reed_sensors[solenoid_num + 8] == 0
        actuate = reed_sensors[solenoid_num] == 0 and reed_sensors[solenoid_num + 8] == 1
        
        if state == 'default' and default:
            log_to_gui(f"Solenoid {solenoid_num+1} verified in default position", "DEBUG")
            return True
        elif state == 'actuate' and actuate:
            log_to_gui(f"Solenoid {solenoid_num+1} verified in actuated position", "DEBUG")
            return True
        
        time.sleep(0.05)  # Small delay between checks
    
    # Timeout reached
    log_to_gui(f"Verification timeout for solenoid {solenoid_num+1}", "WARNING")
    return False

def actuate_solenoid(solenoid_num):
    """Actuate a solenoid and verify position"""
    log_to_gui(f"Actuating solenoid {solenoid_num+1}", "INFO")
    solenoid_states[solenoid_num] = 1
    write_solenoids()
    root.after(int(SOLENOID_ACTUATION_TIME * 1000), lambda: check_actuation(solenoid_num))

def check_actuation(solenoid_num):
    """Check if solenoid actuated successfully"""
    if verify_state(solenoid_num, 'actuate'):
        root.after(int(SOLENOID_HOLD_TIME * 1000), lambda: reset_solenoid(solenoid_num))
    else:
        error_msg = f"Solenoid {solenoid_num+1} failed to actuate."
        errors.append(error_msg)
        log_to_gui(error_msg, "ERROR")
        update_gui()

def reset_solenoid(solenoid_num):
    """Reset solenoid to default position"""
    log_to_gui(f"Resetting solenoid {solenoid_num+1} to default", "INFO")
    solenoid_states[solenoid_num] = 0
    write_solenoids()
    root.after(int(SOLENOID_RESET_TIME * 1000), lambda: settle_and_verify(solenoid_num))

def settle_and_verify(solenoid_num):
    """Wait for settling then verify default position"""
    root.after(int(SOLENOID_SETTLE_TIME * 1000), lambda: final_verification(solenoid_num))

def final_verification(solenoid_num):
    """Final verification of solenoid return to default"""
    if not verify_state(solenoid_num, 'default'):
        error_msg = f"Solenoid {solenoid_num+1} failed to return to default state."
        errors.append(error_msg)
        log_to_gui(error_msg, "ERROR")
    root.after(int(VERIFICATION_DELAY * 1000), update_gui)

# ========== AUTO MODE SEQUENCE EXECUTION ==========
def execute_sequence_step_with_feedback(step, callback=None):
    """Execute one step of the sequence with feedback validation"""
    global stop_auto
    
    # Check if auto mode was stopped
    if stop_auto:
        log_to_gui("Auto mode stopped - aborting sequence", "WARNING")
        bring_all_to_default()
        return
    
    if step >= len(sequence):
        log_to_gui(f"Sequence completed ({len(sequence)} steps)", "INFO")
        if callback:
            callback()
        return

    log_to_gui(f"Executing sequence step {step+1}/{len(sequence)}", "INFO")
    
    # Set solenoid states for this step
    solenoid_states[:] = sequence[step]
    write_solenoids()
    update_gui()
    
    # Wait for solenoids to actuate, then validate
    root.after(int(STEP_VALIDATION_DELAY * 1000), lambda: validate_feedback_for_step(step, callback))

def validate_feedback_for_step(step, callback):
    """Validate reed sensor feedback for current step"""
    global stop_auto
    
    # Check if auto mode was stopped
    if stop_auto:
        log_to_gui("Auto mode stopped - aborting validation", "WARNING")
        bring_all_to_default()
        return
    
    reed_sensors = read_reed_sensors()
    step_passed = True
    
    log_to_gui(f"Validating feedback for step {step+1}", "INFO")
    
    for solenoid_num, state in enumerate(sequence[step]):
        if state == 1:
            actuate = reed_sensors[solenoid_num] == 0 and reed_sensors[solenoid_num + 8] == 1
            if not actuate:
                error_msg = f"[Step {step+1}] Solenoid {solenoid_num+1} failed feedback check."
                errors.append(error_msg)
                log_to_gui(error_msg, "ERROR")
                step_passed = False

    if step_passed:
        log_to_gui(f"Step {step+1} validation passed", "INFO")
        next_step = step + 1
        root.after(int(NEXT_STEP_DELAY * 1000), lambda: execute_sequence_step_with_feedback(next_step, callback))
    else:
        log_to_gui(f"Step {step+1} validation failed - stopping auto mode", "ERROR")
        stop_auto_mode()
        update_gui()

def auto_mode():
    """Start automatic operation mode"""
    global stop_auto, manual_mode_active, auto_cycle_count
    
    if not stop_auto:
        log_to_gui("Auto mode already running", "WARNING")
        return
    
    stop_auto = False
    manual_mode_active = False
    auto_cycle_count = 0
    
    log_to_gui("=" * 70, "INFO")
    log_to_gui("AUTO MODE STARTED", "INFO")
    log_to_gui("=" * 70, "INFO")
    
    update_gui()
    
    # Start the first cycle
    start_auto_cycle()

def start_auto_cycle():
    """Start a single auto cycle (sequence + motor)"""
    global stop_auto, auto_cycle_count
    
    if stop_auto:
        log_to_gui("Auto mode stopped - not starting new cycle", "INFO")
        return
    
    auto_cycle_count += 1
    log_to_gui(f"Starting auto cycle #{auto_cycle_count}", "INFO")
    log_to_gui("-" * 70, "INFO")
    
    # Execute sequence, then run motor
    execute_sequence_step_with_feedback(0, on_sequence_complete)

def on_sequence_complete():
    """Called when sequence completes - now run the motor"""
    global stop_auto
    
    if stop_auto:
        log_to_gui("Auto mode stopped after sequence - not running motor", "INFO")
        bring_all_to_default()
        return
    
    log_to_gui("Sequence complete - starting motor movement", "INFO")
    run_stepper_motor(callback=on_motor_complete)

def on_motor_complete():
    """Called when motor completes - restart the cycle"""
    global stop_auto, auto_cycle_count
    
    if stop_auto:
        log_to_gui(f"Auto mode stopped after motor - completed {auto_cycle_count} cycles", "INFO")
        bring_all_to_default()
        return
    
    log_to_gui(f"Cycle #{auto_cycle_count} complete", "INFO")
    log_to_gui(f"Waiting {CYCLE_RESTART_DELAY}s before next cycle...", "INFO")
    
    # Wait before starting next cycle
    root.after(int(CYCLE_RESTART_DELAY * 1000), start_auto_cycle)

def stop_auto_mode():
    """Stop automatic operation mode"""
    global stop_auto, auto_cycle_count
    stop_auto = True
    log_to_gui("=" * 70, "WARNING")
    log_to_gui(f"AUTO MODE STOPPED (Completed {auto_cycle_count} cycles)", "WARNING")
    log_to_gui("=" * 70, "WARNING")
    update_gui()

# ========== MANUAL MODE FUNCTIONS ==========
def manual_mode():
    """Activate manual mode"""
    global manual_mode_active, stop_auto
    stop_auto = True
    manual_mode_active = True
    bring_all_to_default()
    log_to_gui("Manual Mode Activated", "INFO")
    update_gui()

def manual_pre_assembly():
    """Manual control of pre-assembly section"""
    if manual_mode_active:
        log_to_gui("Manual Pre-assembly sequence started", "INFO")
        actuate_manual_sequence(0, 3)
    else:
        log_to_gui("Manual mode not active - activate manual mode first", "WARNING")

def manual_main_assembly():
    """Manual control of main assembly section"""
    if manual_mode_active:
        log_to_gui("Manual Main-assembly sequence started", "INFO")
        actuate_manual_sequence(3, 6)
    else:
        log_to_gui("Manual mode not active - activate manual mode first", "WARNING")
        
def manual_post_assembly():
    """Manual control of post-assembly section"""
    if manual_mode_active:
        log_to_gui("Manual Post-assembly sequence started", "INFO")
        actuate_manual_sequence(6, 8)
    else:
        log_to_gui("Manual mode not active - activate manual mode first", "WARNING")

def actuate_manual_sequence(start, end):
    """Execute sequence for specific solenoid range in manual mode"""
    log_to_gui(f"Actuating solenoids {start+1} to {end}", "INFO")
    
    def actuate_next_manual_step(step):
        if step < len(sequence):
            log_to_gui(f"Manual sequence step {step+1}/{len(sequence)}", "DEBUG")
            for i in range(start, end):
                solenoid_states[i] = sequence[step][i]
            write_solenoids()
            update_gui()
            root.after(400, lambda: actuate_next_manual_step(step + 1))
        else:
            log_to_gui("Manual sequence completed - returning to default", "INFO")
            bring_all_to_default()
    
    actuate_next_manual_step(0)

# ========== STEPPER MOTOR CONTROL ==========
def run_stepper_motor(callback=None):
    """
    Run stepper motor with non-blocking execution
    Monitors thru-beam sensor for homing
    """
    global motor_running, motor_speed, last_sensor_state
    
    if motor_running:
        error_msg = "Motor already running - command ignored"
        log_to_gui(error_msg, "WARNING")
        errors.append(error_msg)
        update_gui()
        return
    
    motor_running = True
    step_delay = 1 / (motor_speed * STEPS_PER_REVOLUTION / 60)  # Delay per step based on speed
    step_count = [0]  # Use list to maintain mutable state in closure
    motor_stopped = [False]
    
    log_to_gui(f"Running stepper motor at {motor_speed}% speed for {STEPS_PER_CYCLE} steps", "INFO")
    
    def step_once():
        """Execute single motor step (non-blocking)"""
        global last_sensor_state, motor_running, stop_auto
        
        # Check if auto mode stopped
        if stop_auto and motor_running:
            motor_running = False
            log_to_gui("Motor stopped due to auto mode stop", "WARNING")
            if callback:
                callback()
            return
        
        if motor_stopped[0]:
            return
        
        if step_count[0] >= STEPS_PER_CYCLE:
            # Motor cycle complete
            motor_running = False
            log_to_gui(f"Motor completed {step_count[0]} steps", "INFO")
            if callback:
                callback()
            return
        
        # Check sensor for edge detection (rising edge: 0 -> 1)
        current_sensor_state = thru_sensor_pin.value
        
        if last_sensor_state == 0 and current_sensor_state == 1:
            # Rising edge detected - product detected
            log_to_gui("Rising edge detected! Product detected - initiating homing adjustment", "INFO")
            motor_stopped[0] = True
            last_sensor_state = current_sensor_state
            
            # Execute homing adjustment after settling time
            root.after(int(HOMING_SETTLE_TIME * 1000), lambda: execute_homing_adjustment(callback))
            return
        
        last_sensor_state = current_sensor_state
        
        # Execute one motor step
        GPIO.output(STEP_PIN, GPIO.HIGH)
        root.after(int(step_delay / 2 * 1000), lambda: complete_step(step_delay, step_count))
    
    def complete_step(step_delay, step_count):
        """Complete the low phase of step pulse"""
        GPIO.output(STEP_PIN, GPIO.LOW)
        step_count[0] += 1
        
        if step_count[0] % 200 == 0:  # Log every 200 steps
            log_to_gui(f"Motor progress: {step_count[0]}/{STEPS_PER_CYCLE} steps", "DEBUG")
        
        # Schedule next step
        root.after(int(step_delay / 2 * 1000), step_once)
    
    # Start stepping
    step_once()

def execute_homing_adjustment(callback):
    """Execute homing adjustment steps after sensor detection"""
    global motor_running, stop_auto
    
    step_delay = 1 / (motor_speed * STEPS_PER_REVOLUTION / 60)
    step_count = [0]
    
    log_to_gui(f"Starting homing adjustment: {adjusted_homing_step} steps", "INFO")
    
    def homing_step():
        """Execute single homing step"""
        global motor_running, stop_auto
        
        # Check if auto mode stopped
        if stop_auto and motor_running:
            motor_running = False
            log_to_gui("Homing stopped due to auto mode stop", "WARNING")
            if callback:
                callback()
            return
        
        if step_count[0] >= adjusted_homing_step:
            log_to_gui("Homing adjustment completed", "INFO")
            motor_running = False
            if callback:
                callback()
            return
        
        GPIO.output(STEP_PIN, GPIO.HIGH)
        root.after(int(step_delay / 2 * 1000), lambda: complete_homing_step(step_delay, step_count))
    
    def complete_homing_step(step_delay, step_count):
        """Complete homing step low phase"""
        GPIO.output(STEP_PIN, GPIO.LOW)
        step_count[0] += 1
        
        if step_count[0] % 50 == 0:  # Log every 50 steps
            log_to_gui(f"Homing progress: {step_count[0]}/{adjusted_homing_step} steps", "DEBUG")
        
        root.after(int(step_delay / 2 * 1000), homing_step)
    
    homing_step()

def toggle_motor_direction():
    """Toggle motor rotation direction"""
    global direction
    direction = GPIO.HIGH if direction == GPIO.LOW else GPIO.LOW
    GPIO.output(DIR_PIN, direction)
    direction_name = 'Clockwise' if direction == GPIO.HIGH else 'Counter-Clockwise'
    log_to_gui(f"Motor direction changed to: {direction_name}", "INFO")
    root.after(int(MOTOR_DIRECTION_SETTLE * 1000), lambda: None)  # Small settle delay

# ========== SYSTEM CONTROL FUNCTIONS ==========
def reset_system():
    """Reset entire system to default state"""
    global stop_auto, manual_mode_active, motor_running, auto_cycle_count
    stop_auto = True
    manual_mode_active = False
    motor_running = False
    auto_cycle_count = 0
    errors.clear()
    bring_all_to_default()
    log_to_gui("=" * 70, "INFO")
    log_to_gui("SYSTEM RESET COMPLETED", "INFO")
    log_to_gui("=" * 70, "INFO")
    update_gui()

def bring_all_to_default():
    """Return all solenoids to default (retracted) state"""
    log_to_gui("Bringing all solenoids to default state", "INFO")
    for solenoid_num in range(8):
        solenoid_states[solenoid_num] = 0
    write_solenoids()
    update_gui()

def emergency_stop():
    """Emergency stop - halt all operations immediately"""
    global stop_auto, manual_mode_active, motor_running, auto_cycle_count
    stop_auto = True
    manual_mode_active = False
    motor_running = False
    bring_all_to_default()
    error_msg = "⚠️ EMERGENCY STOP ACTIVATED ⚠️"
    errors.append(error_msg)
    log_to_gui("=" * 70, "ERROR")
    log_to_gui(error_msg, "ERROR")
    log_to_gui(f"System halted after {auto_cycle_count} cycles", "ERROR")
    log_to_gui("=" * 70, "ERROR")
    update_gui()

def clear_log():
    """Clear the GUI log display"""
    log_text.delete(1.0, tk.END)
    log_to_gui("Log cleared", "INFO")

def cleanup():
    """Cleanup GPIO and hardware before exit"""
    log_to_gui("Cleaning up system resources...", "INFO")
    bring_all_to_default()
    GPIO.cleanup()
    log_to_gui("Cleanup completed - system shutdown", "INFO")

# ========== GUI SETUP ==========
root = tk.Tk()
root.geometry("800x480")
root.title("PLC-Like Solenoid Control GUI")
root.protocol("WM_DELETE_WINDOW", lambda: (cleanup(), root.destroy()))

# Make GUI responsive
for i in range(4):
    root.columnconfigure(i, weight=1)
for i in range(15):
    root.rowconfigure(i, weight=1)

# ========== GUI LABELS ==========
pre_labels = []
main_labels = []
post_labels = []

# Pre-assembly section (Solenoids 1-3)
tk.Label(root, text="Pre-assembly", font=("Helvetica", 12, "bold")).grid(row=0, column=0, pady=5)
for i in range(3):
    lbl = tk.Label(root, text=f"Solenoid {i+1}: Default", width=20, bg="green", anchor="w", font=("Helvetica", 10))
    lbl.grid(row=i+1, column=0, padx=5, pady=5)
    pre_labels.append(lbl)

# Main assembly section (Solenoids 4-6)
tk.Label(root, text="Main assembly", font=("Helvetica", 12, "bold")).grid(row=0, column=1, pady=5)
for i in range(3):
    lbl = tk.Label(root, text=f"Solenoid {i+4}: Default", width=20, bg="green", anchor="w", font=("Helvetica", 10))
    lbl.grid(row=i+1, column=1, padx=5, pady=5)
    main_labels.append(lbl)

# Post-assembly section (Solenoids 7-8)
tk.Label(root, text="Post-assembly", font=("Helvetica", 12, "bold")).grid(row=0, column=2, pady=5)
for i in range(2):
    lbl = tk.Label(root, text=f"Solenoid {i+7}: Default", width=20, bg="green", anchor="w", font=("Helvetica", 10))
    lbl.grid(row=i+1, column=2, padx=5, pady=5)
    post_labels.append(lbl)

# ========== STATUS DISPLAY ==========
status_label = tk.Label(root, text="Status: Idle", font=("Helvetica", 11, "bold"), 
                        bg="gray", fg="white", relief=tk.RAISED, padx=10, pady=5)
status_label.grid(row=0, column=3, pady=5, padx=5, sticky="ew")

cycle_count_label = tk.Label(root, text="Cycles: 0", font=("Helvetica", 10), 
                              bg="lightgray", relief=tk.SUNKEN, padx=5, pady=3)
cycle_count_label.grid(row=1, column=3, pady=2, padx=5, sticky="ew")

# ========== ERROR DISPLAY ==========
tk.Label(root, text="Error Log", font=("Helvetica", 12, "bold")).grid(row=7, column=0, columnspan=3, pady=5)
error_listbox = tk.Listbox(root, height=4, width=80, font=("Helvetica", 9))
error_listbox.grid(row=8, columnspan=3, padx=10, pady=5)

# ========== SYSTEM LOG DISPLAY ==========
tk.Label(root, text="System Log", font=("Helvetica", 12, "bold")).grid(row=11, column=0, columnspan=3, pady=5)
log_frame = tk.Frame(root)
log_frame.grid(row=12, column=0, columnspan=4, padx=10, pady=5, sticky="nsew")

log_text = scrolledtext.ScrolledText(log_frame, height=8, width=95, font=("Courier", 9), 
                                      bg="black", fg="lime", wrap=tk.WORD)
log_text.pack(fill=tk.BOTH, expand=True)

# Clear log button
tk.Button(root, text="Clear Log", command=clear_log, font=("Helvetica", 9), 
          bg="#607D8B", fg="white").grid(row=13, column=3, pady=2, padx=5, sticky="ew")

# ========== UPDATE GUI FUNCTION ==========
def update_gui():
    """Update GUI to reflect current solenoid states and errors"""
    global stop_auto, auto_cycle_count
    
    # Update solenoid status labels
    for i, lbl in enumerate(pre_labels):
        state_text = "Actuated" if solenoid_states[i] == 1 else "Default"
        lbl.config(
            text=f"Solenoid {i+1}: {state_text}",
            bg="red" if solenoid_states[i] == 1 else "green"
        )
    
    for i, lbl in enumerate(main_labels):
        state_text = "Actuated" if solenoid_states[i+3] == 1 else "Default"
        lbl.config(
            text=f"Solenoid {i+4}: {state_text}",
            bg="red" if solenoid_states[i+3] == 1 else "green"
        )
    
    for i, lbl in enumerate(post_labels):
        state_text = "Actuated" if solenoid_states[i+6] == 1 else "Default"
        lbl.config(
            text=f"Solenoid {i+7}: {state_text}",
            bg="red" if solenoid_states[i+6] == 1 else "green"
        )
    
    # Update status label
    if not stop_auto:
        status_label.config(text="Status: AUTO RUNNING", bg="green")
    elif manual_mode_active:
        status_label.config(text="Status: MANUAL MODE", bg="orange")
    else:
        status_label.config(text="Status: Idle", bg="gray")
    
    # Update cycle count
    cycle_count_label.config(text=f"Cycles: {auto_cycle_count}")
    
    # Update error list
    error_listbox.delete(0, tk.END)
    for error in errors:
        error_listbox.insert(tk.END, error)

# ========== MOTOR SPEED CONTROL ==========
speed_label = tk.Label(root, text=f"Motor Speed: {motor_speed}%", font=("Helvetica", 11))
speed_label.grid(row=9, column=0, columnspan=2, padx=5, pady=5)

def set_motor_speed(value):
    """Update motor speed from slider"""
    global motor_speed
    motor_speed = int(float(value))
    speed_label.config(text=f"Motor Speed: {motor_speed}%")
    log_to_gui(f"Motor speed adjusted to {motor_speed}%", "INFO")

speed_scale = tk.Scale(root, from_=10, to=100, orient=tk.HORIZONTAL, 
                       command=set_motor_speed, length=300, font=("Helvetica", 10))
speed_scale.set(motor_speed)
speed_scale.grid(row=10, column=0, columnspan=2, padx=5, pady=5)

# ========== HOMING ADJUSTMENT CONTROL ==========
homing_label = tk.Label(root, text=f"Homing Adjust: {adjusted_homing_step} steps", font=("Helvetica", 11))
homing_label.grid(row=9, column=2, columnspan=2, padx=5, pady=5)

def set_homing_adjustment(value):
    """Update homing adjustment from slider"""
    global adjusted_homing_step
    adjusted_homing_step = int(float(value))
    homing_label.config(text=f"Homing Adjust: {adjusted_homing_step} steps")
    log_to_gui(f"Homing adjustment set to {adjusted_homing_step} steps", "INFO")

homing_scale = tk.Scale(root, from_=0, to=500, orient=tk.HORIZONTAL, 
                        command=set_homing_adjustment, length=300, font=("Helvetica", 10))
homing_scale.set(adjusted_homing_step)
homing_scale.grid(row=10, column=2, columnspan=2, padx=5, pady=5)

# ========== CONTROL BUTTONS ==========
# Row 4 - Main control buttons
tk.Button(root, text="Manual Mode", command=manual_mode, font=("Helvetica", 11), 
          bg="#4CAF50", fg="white").grid(row=4, column=0, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Auto Mode", command=auto_mode, font=("Helvetica", 11), 
          bg="#2196F3", fg="white").grid(row=4, column=1, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Stop Auto", command=stop_auto_mode, font=("Helvetica", 11), 
          bg="#FF9800", fg="white").grid(row=4, column=2, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Reset System", command=reset_system, font=("Helvetica", 11), 
          bg="#9E9E9E", fg="white").grid(row=4, column=3, pady=5, padx=5, sticky="ew")

# Row 5 - Manual section controls
tk.Button(root, text="Pre-assembly", command=manual_pre_assembly, font=("Helvetica", 10), 
          bg="#8BC34A").grid(row=5, column=0, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Main assembly", command=manual_main_assembly, font=("Helvetica", 10), 
          bg="#8BC34A").grid(row=5, column=1, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Post-assembly", command=manual_post_assembly, font=("Helvetica", 10), 
          bg="#8BC34A").grid(row=5, column=2, pady=5, padx=5, sticky="ew")
tk.Button(root, text="Run Stepper Motor", command=lambda: run_stepper_motor(), font=("Helvetica", 10), 
          bg="#03A9F4").grid(row=5, column=3, pady=5, padx=5, sticky="ew")

# Row 6 - Additional controls
tk.Button(root, text="Toggle Motor Direction", command=toggle_motor_direction, font=("Helvetica", 10), 
          bg="#00BCD4").grid(row=6, column=3, pady=5, padx=5, sticky="ew")

# Emergency Stop Button - Prominent placement
tk.Button(root, text="🛑 EMERGENCY STOP", command=emergency_stop, 
          bg="red", fg="white", font=("Helvetica", 14, "bold"), 
          height=2).grid(row=14, column=0, columnspan=4, pady=10, padx=10, sticky="ew")

# ========== SYSTEM INITIALIZATION ==========
log_to_gui("=" * 70, "INFO")
log_to_gui("MACHINE CONTROL SYSTEM INITIALIZING", "INFO")
log_to_gui("=" * 70, "INFO")
log_to_gui("Hardware: Raspberry Pi + MCP23017 I/O Expanders", "INFO")
log_to_gui(f"Configuration: {len(solenoid_pins)} solenoids, {len(reed_sensor_pins)} sensors", "INFO")
log_to_gui(f"Motor: {STEPS_PER_REVOLUTION} steps/rev, {STEPS_PER_CYCLE} steps/cycle", "INFO")
bring_all_to_default()
log_to_gui("System ready for operation", "INFO")
log_to_gui("=" * 70, "INFO")
update_gui()

# ========== START GUI ==========
root.mainloop()