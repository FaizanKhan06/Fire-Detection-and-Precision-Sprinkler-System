import cv2
import picamera
import picamera.array
from threading import Thread
import time

# Load the pre-trained face detection classifier
face_cascade = cv2.CascadeClassifier('/home/pi/Downloads/haarcascade_frontalface_default.xml')

# Initialize the Picamera
camera = picamera.PiCamera()
camera.resolution = (320, 240)
camera.framerate = 10  # Adjust frame rate as needed

# Function for face detection
def detect_faces(frame):
    """
    Detect faces in a frame and draw rectangles around them.
    
    Args:
        frame: The frame in which to detect faces.
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=8, minSize=(30, 30))

    # Draw rectangles around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw a 5 pixel wide point at the center of the rectangle
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
        print("Face Detected")

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
