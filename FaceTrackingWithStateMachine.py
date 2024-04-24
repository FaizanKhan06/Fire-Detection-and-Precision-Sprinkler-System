import cv2
import time
import math
import smtplib
from email.mime.text import MIMEText
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO

class StateMachine:
    def __init__(self):
        self.state = "Finding State"
        self.last_state_change_time = time.time()
        self.total_frames = 0
        self.face_detected_frames = 0
        self.found_state_duration = 5  # Duration for staying in Found State
        self.emailing_state_duration = 10  # Duration for staying in Emailing State
        self.extinguishing_state_duration = 5  # Duration for staying in Start Extinguishing State
        self.face_percent_min = 10  # Minimum face percentage to start extinguisher
        self.email_sent = False

    def send_email(self):
        print("Sending email")
        """
        sender_email = "1hk20cs045@hkbk.edu.in"  # Replace with your email
        receiver_email = "faizankhanm062002@gmail.com"  # Replace with recipient email
        password = "cs205554"  # Replace with your email password
        
        subject = "Face detected!"
        body = "A face has been detected. Please take action."
        
        msg = MIMEText(body)
        msg['Subject'] = subject
        msg['From'] = sender_email
        msg['To'] = receiver_email
        
        server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
        server.login(sender_email, password)
        server.send_message(msg)
        server.quit()
        """

    def transition(self, event):
        current_time = time.time()
        elapsed_time = current_time - self.last_state_change_time

        if self.state == "Finding State":
            if event == "Face Found":
                self.state = "Found State"
                print("Current State: " + self.state) # print
                self.last_state_change_time = current_time
        elif self.state == "Found State":
            if elapsed_time >= self.found_state_duration:
                if self.total_frames > 0:
                    percentage = (self.face_detected_frames / self.total_frames) * 100
                    print("Percentage of frames with face detected:", percentage)
                    if percentage < self.face_percent_min:
                        self.state = "Finding State"
                        print("Current State: " + self.state) # print
                    else:
                        self.state = "Emailing State"
                        print("Current State: " + self.state) # print
                        self.send_email()  # Send email when entering Emailing State
                        self.email_sent = True
                else:
                    self.state = "Finding State"
                    print("Current State: " + self.state) # print
                self.last_state_change_time = current_time
            else:
                self.total_frames += 1
                if event == "Face Found":
                    self.face_detected_frames += 1
        elif self.state == "Emailing State":
            if elapsed_time >= self.emailing_state_duration:
                self.state = "Start Extinguishing State"
                print("Current State: " + self.state) # print
                self.last_state_change_time = current_time
                self.email_sent = False  # Reset email_sent flag
            elif event == "Back to Finding":
                self.state = "Finding State"
                print("Current State: " + self.state) # print
                self.last_state_change_time = current_time
                self.email_sent = False  # Reset email_sent flag
        elif self.state == "Start Extinguishing State":
            if elapsed_time >= self.extinguishing_state_duration:
                self.state = "Finding State"
                print("Current State: " + self.state) # print
                self.last_state_change_time = current_time
                self.total_frames = 0
                self.face_detected_frames = 0

    def get_state(self):
        return self.state

class Servos:
    def __init__(self):
        self.pi_gpio_factory = PiGPIOFactory()
        # Create AngularServo instance with PiGPIOFactory
        self.pan_servo = AngularServo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=self.pi_gpio_factory)
        self.tilt_servo = AngularServo(23, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=self.pi_gpio_factory)
        self.pan_servo_angle = 8
        self.tilt_servo_angle = 25
        
        self.sweep_variable = 0
        
        self.move_pan_servo()
        self.move_tilt_servo()

    def sweep_motion(self):
        self.sweep_variable += 5
        self.sweep_variable %= 360
        
        self.pan_servo_angle = 90*math.sin(math.radians(self.sweep_variable))
        self.move_pan_servo()
        
        self.tilt_servo_angle = 25
        self.move_tilt_servo()

    # Map Values Between 2 Ranges
    def map_value(self, value, from_min, from_max, to_min, to_max):
        # Check for division by zero
        if from_max - from_min == 0:
            raise ValueError("The 'from_max' value must be different from 'from_min'")

        # Calculate the percentage of value's position in the original range
        percentage = (value - from_min) / (from_max - from_min)

        # Map the percentage to the target range
        mapped_value = to_min + percentage * (to_max - to_min)

        return int(mapped_value)

    def point_motion(self, face_center_x, face_center_y):
        #pan Servo Movement
        if not(-10 <= face_center_x <= 10):
            result = 1 if face_center_x > 0 else -1
            self.pan_servo_angle += (self.map_value(face_center_x*result, 5, 160, 1, 10) * result)
        
        #tilt Servo Movement
        if not(-10 <= face_center_y <= 10):
            result = 1 if face_center_y > 0 else -1
            self.tilt_servo_angle += (self.map_value(face_center_y*result, 5, 120, 1, 10) * result)

        self.move_pan_servo()
        self.move_tilt_servo()

    def initial_positon(self):
        self.pan_servo_angle = 8
        self.tilt_servo_angle = 25
        self.move_pan_servo()
        self.move_tilt_servo()
    
    def move_pan_servo(self):
        self.pan_servo.angle = self.pan_servo_angle
        
    def move_tilt_servo(self):
        self.tilt_servo.angle = self.tilt_servo_angle

class Other_components:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Set pin 24 and 25 as output
        GPIO.setup(24, GPIO.OUT) #Relay 
        GPIO.setup(25, GPIO.OUT) #Buzzer

        # Intially Output of the 24 and 25 pin is Low
        self.stop_buzzer_and_waterpump()

        self.playSound = True

    def start_buzzer_and_waterpump(self):
        if self.playSound:
            self.start_buzzer_sound()
        self.start_water_pump()

    def stop_buzzer_and_waterpump(self):
        self.stop_buzzer_sound()
        self.stop_water_pump()

    def start_buzzer_sound(self):
        GPIO.output(25, GPIO.HIGH)

    def start_water_pump(self):
        GPIO.output(24, GPIO.HIGH)

    def stop_buzzer_sound(self):
        GPIO.output(25, GPIO.LOW)

    def stop_water_pump(self):
        GPIO.output(24, GPIO.LOW)

def main():
    # Initialize state machine
    sm = StateMachine()
    servo = Servos()
    other_components = Other_components()

    # Load pre-trained Haar Cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier('/home/pi/Downloads/haarcascade_frontalface_default.xml')

    # Open the camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 10)

    print("Current State:", sm.get_state())

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame")
            break

        # Vertically flip the frame
        frame = cv2.flip(frame, 0)

        height, width, _ = frame.shape

        # Convert frame to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the grayscale frame
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

        # If faces are found, trigger "Face Found" event
        if len(faces) > 0:
            sm.transition("Face Found")

        # Check for state transition to "Start Extinguishing State"
        if sm.get_state() == "Found State":
            sm.transition("Timeout")

        # Check for state transition to "Emailing State"
        if sm.get_state() == "Found State" and not sm.email_sent:
            sm.transition("Email")

        # Check for state transition back to "Finding State"
        if sm.get_state() == "Start Extinguishing State":
            sm.transition("Timeout")
                
        # Draw rectangles around detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            face_center_x = (center_x - width//2)*1
            face_center_y = (center_y - height//2)*-1
            
            if sm.get_state() == "Found State" or sm.get_state() == "Start Extinguishing State" or sm.get_state() == "Emailing State":
                servo.point_motion(face_center_x, face_center_y)
            #coordinates_text = f"({face_center_x}, {face_center_y})"
            #cv2.putText(frame, coordinates_text, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

        #State Management
        if sm.get_state() == "Finding State":
            servo.sweep_motion()
            other_components.stop_buzzer_and_waterpump()
        if sm.get_state() == "Found State" or sm.get_state() == "Start Extinguishing State" or sm.get_state() == "Emailing State":
            if sm.get_state() == "Start Extinguishing State":
                other_components.start_buzzer_and_waterpump()

        cv2.putText(frame, sm.get_state(), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow('Face Detection', frame)

        # Check for key press "s" to transition back to "Finding State" from "Emailing State"
        key = cv2.waitKey(1)
        if key & 0xFF == ord('s') and sm.get_state() == "Emailing State":
            sm.transition("Back to Finding")
        if key & 0xFF == ord('q'):
            other_components.stop_buzzer_and_waterpump()
            servo.initial_positon()
            break

    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
