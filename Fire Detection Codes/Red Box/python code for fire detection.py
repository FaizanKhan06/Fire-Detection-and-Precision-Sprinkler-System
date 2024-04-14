import cv2

fire_cascade = cv2.CascadeClassifier('cascade.xml')

cap = cv2.VideoCapture(0) #start video capturing

while cap.isOpened():
    ret, img = cap.read() #capture a frame
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #convert image to grayscale
    fire = fire_cascade.detectMultiScale(img, 12, 5) #test for fire detection
    for (x,y,w,h) in fire:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2) #highlight the area of image with fire
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        
    cv2.imshow('img', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

