# gui.py
import tkinter as tk
import state
from sequence import execute_step
from stepper import run

def start_gui():
    root = tk.Tk()
    root.geometry("800x480")
    root.title("Omni Lens Controller")

    tk.Button(root, text="Auto Start",
              command=lambda: execute_step(0)).pack()

    tk.Button(root, text="Run Motor",
              command=lambda: run(984)).pack()

    root.mainloop()
