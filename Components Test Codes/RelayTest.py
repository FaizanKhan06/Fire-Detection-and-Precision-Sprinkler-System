import tkinter as tk
import RPi.GPIO as GPIO

# Suppress warnings
GPIO.setwarnings(False)

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set pin 24 as output
GPIO.setup(24, GPIO.OUT)

def turn_on_relay():
    GPIO.output(24, GPIO.HIGH)
    print("Relay turned on")

def turn_off_relay():
    GPIO.output(24, GPIO.LOW)
    print("Relay turned off")

# Function to handle button click
def toggle_relay():
    if GPIO.input(24) == GPIO.LOW:
        turn_on_relay()
        button.config(text="Turn Off Relay")
    else:
        turn_off_relay()
        button.config(text="Turn On Relay")

# Create tkinter window
window = tk.Tk()
window.title("Relay Control")

# Create button
button = tk.Button(window, text="Turn On Relay", width=15, command=toggle_relay)
button.pack(pady=20)

# Run the tkinter event loop
window.mainloop()

# Clean up GPIO
GPIO.cleanup()

