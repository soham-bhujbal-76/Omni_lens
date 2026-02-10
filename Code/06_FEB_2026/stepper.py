# stepper.py
import RPi.GPIO as GPIO
import time
import state
from config import *

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

def run(steps):
    delay = 1 / (state.motor_speed * STEPS_PER_REV / 60)
    for _ in range(steps):
        GPIO.output(STEP_PIN, 1)
        time.sleep(delay/2)
        GPIO.output(STEP_PIN, 0)
        time.sleep(delay/2)
