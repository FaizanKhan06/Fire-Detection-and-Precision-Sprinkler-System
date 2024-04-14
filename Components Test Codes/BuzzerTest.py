import tkinter as tk
import RPi.GPIO as GPIO

# Suppress warnings
GPIO.setwarnings(False)

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set pin 25 as output
GPIO.setup(25, GPIO.OUT)

def turn_on_buzzer():
    GPIO.output(25, GPIO.HIGH)
    print("Buzzer turned on")

def turn_off_buzzer():
    GPIO.output(25, GPIO.LOW)
    print("Buzzer turned off")

# Function to handle button click
def toggle_buzzer():
    if GPIO.input(25) == GPIO.LOW:
        turn_on_buzzer()
        button.config(text="Turn Off Buzzer")
    else:
        turn_off_buzzer()
        button.config(text="Turn On Buzzer")

# Create tkinter window
window = tk.Tk()
window.title("Buzzer Control")

# Create button
button = tk.Button(window, text="Turn On Buzzer", width=15, command=toggle_buzzer)
button.pack(pady=20)

# Run the tkinter event loop
window.mainloop()

# Clean up GPIO
GPIO.cleanup()
