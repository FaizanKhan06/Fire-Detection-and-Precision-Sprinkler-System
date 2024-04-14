from gpiozero import AngularServo
from tkinter import *
from gpiozero.pins.pigpio import PiGPIOFactory

# Set up PiGPIOFactory
pi_gpio_factory = PiGPIOFactory()

# Create Tkinter GUI
root = Tk()
root.title("Servo Control")

# Create AngularServo instance with PiGPIOFactory
servo = AngularServo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=pi_gpio_factory)  # Assuming servo is connected to GPIO 18
#servo = AngularServo(23, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=pi_gpio_factory) 
# Function to update servo position
def update_servo_position(value):
    angle = int(value)   # Map slider value to -90 to 90 range
    servo.angle = angle

# Slider widget
slider = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=update_servo_position)
slider.pack(pady=10)

# Run the Tkinter event loop
root.mainloop()
