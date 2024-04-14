import picamera
import io
import cv2
import numpy as np

def main():
    # Initialize the PiCamera
    camera = picamera.PiCamera()

    try:
        # Set camera resolution (you can adjust as needed)
        camera.resolution = (640, 480)

        # Create an in-memory stream to capture the image data
        stream = io.BytesIO()

        # Capture continuously from the camera
        for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            # Reset the stream position to the beginning
            stream.seek(0)

            # Read the image data from the stream
            data = np.frombuffer(stream.getvalue(), dtype=np.uint8)

            # Decode the image
            image = cv2.imdecode(data, 1)

            # Flip the image vertically
            image = cv2.flip(image, 0)

            # Display the image
            cv2.imshow('Live Feed', image)
            key = cv2.waitKey(1) & 0xFF

            # If the 'q' key is pressed, break from the loop
            if key == ord('q'):
                break

            # Reset the stream for the next capture
            stream.seek(0)
            stream.truncate()

    finally:
        # Close the OpenCV window
        cv2.destroyAllWindows()

        # Close the PiCamera connection
        camera.close()

if __name__ == "__main__":
    main()
