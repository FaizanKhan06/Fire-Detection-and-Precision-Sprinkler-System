import cv2
import picamera
import picamera.array
from threading import Thread
import time
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO

# Suppress warnings
GPIO.setwarnings(False)

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set pin 25 as output
GPIO.setup(25, GPIO.OUT)

# Set pin 24 as output
GPIO.setup(24, GPIO.OUT)

pi_gpio_factory = PiGPIOFactory()

# Create AngularServo instance with PiGPIOFactory
pan_servo = AngularServo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=pi_gpio_factory)
tilt_servo = AngularServo(23, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=pi_gpio_factory)
pan_servo_angle = 0
tilt_servo_angle = 0

# Load the pre-trained face detection classifier
face_cascade = cv2.CascadeClassifier('/home/pi/Downloads/haarcascade_frontalface_default.xml')

# Initialize the Picamera
camera = picamera.PiCamera()
camera.resolution = (320, 240)
camera.framerate = 10  # Adjust frame rate as needed

def map_value(value, from_min, from_max, to_min, to_max):
    """
    Maps a value from one range to another range.

    Parameters:
        value (float): The value to be mapped.
        from_min (float): The minimum value of the original range.
        from_max (float): The maximum value of the original range.
        to_min (float): The minimum value of the target range.
        to_max (float): The maximum value of the target range.

    Returns:
        float: The mapped value.
    """
    # Check for division by zero
    if from_max - from_min == 0:
        raise ValueError("The 'from_max' value must be different from 'from_min'")

    # Calculate the percentage of value's position in the original range
    percentage = (value - from_min) / (from_max - from_min)

    # Map the percentage to the target range
    mapped_value = to_min + percentage * (to_max - to_min)

    return int(mapped_value)

# Initialize pan-tilt servo motors
def calc_pan_tilt(face_center_x, face_center_y):
    global pan_servo_angle, tilt_servo_angle
    
    #check if pan servo in center
    if not(-10 <= face_center_x <= 10):
        result = 1 if face_center_x > 0 else -1
        pan_servo_angle += (map_value(face_center_x*result, 5, 160, 1, 10) * result)
        #print(map_value(face_center_x*result, 5, 160, 1, 10))
        
    #check if pan servo in center
    if not(-10 <= face_center_y <= 10):
        result = 1 if face_center_y > 0 else -1
        tilt_servo_angle += (map_value(face_center_y*result, 5, 120, 1, 10) * result)

    # Move the pan-tilt mechanism
    pan_servo.angle = pan_servo_angle
    tilt_servo.angle = tilt_servo_angle
    
def face_detected(face_center_x,face_center_y):
    calc_pan_tilt(face_center_x,face_center_y)
    GPIO.output(24, GPIO.LOW) #Relay
    #GPIO.output(25, GPIO.HIGH) #Buzzer
    
def face_not_detected():
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.LOW)

# Function for face detection
def detect_faces(frame):
    """
    Detect faces in a frame and draw rectangles around them.
    
    Args:
        frame: The frame in which to detect faces.
    """
    height, width, _ = frame.shape

    # Draw horizontal and vertical lines at the center of the screen
    #cv2.line(frame, (0, height//2), (width, height//2), (255, 255, 255), 1)  # Horizontal line
    #cv2.line(frame, (width//2, 0), (width//2, height), (255, 255, 255), 1)  # Vertical line
    #cv2.rectangle(frame, (frame.shape[1]//2 - 10, frame.shape[0]//2 - 10), (frame.shape[1]//2 + 10, frame.shape[0]//2 + 10), (0, 0, 255), 2)
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame
    faces = face_cascade.detectMultiScale(gray,1.1, 6)
    
    if len(faces) == 0:
        face_not_detected()

    # Draw rectangles around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw a 5 pixel wide point at the center of the rectangle
        #cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
        # Print the coordinates of the blue circle
        face_center_x = (center_x - width//2)*1
        face_center_y = (center_y - height//2)*-1
        
        coordinates_text = f"({face_center_x}, {face_center_y})"
        cv2.putText(frame, coordinates_text, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        face_detected(face_center_x,face_center_y)
        #print("Face Detected")

    # Display the resulting frame
    cv2.imshow('Face Detection', frame)

# Thread for capturing frames and performing face detection
def capture_frames():
    """
    Continuously capture frames from the camera and perform face detection.
    """
    for _ in camera.capture_continuous(picamera.array.PiRGBArray(camera), format='bgr', use_video_port=True):
        output = _.array

        # Vertically flip the frame
        frame = cv2.flip(output, 0)

        # Perform face detection
        detect_faces(frame)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pan_servo.angle = 0
            tilt_servo.angle = 0
            GPIO.output(25, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)
            break

        # Clear the stream in preparation for the next frame
        _.truncate(0)

# Start the thread for capturing frames and performing face detection
thread = Thread(target=capture_frames)
thread.start()

# Wait for the thread to finish
thread.join()

# Clean up resources
camera.close()
cv2.destroyAllWindows()

