#! /usr/bin/env python3
import rospy
import cv2
import face_recognition
from std_msgs.msg import Int16
import os

# Camera URL (Video URL not the whole server)
stream_url = "http://192.168.121.30:81/stream"

# Trusted Faces List
# Define face images and names
faces_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../faces")
image_files = ["captured_face.jpg"]  # Add all image filenames here
knownNames = ["MWA"] # A name for each trusted face

# Load images
knownFaces = [face_recognition.load_image_file(os.path.join(faces_dir, img)) for img in image_files]
knownFacesEncodings = [face_recognition.face_encodings(image)[0] for image in knownFaces] # Encodings for each face

# Counters
counter = 0 # Frame Counter
facesCounter = len(knownFaces) + 1 #Faces Counter (Initialized with the number of known faces)
message = 0
lastMessage = 90
stream_error_counter = 0

class Camera:
    def __init__(self):
        rospy.init_node("CameraStream")
        self.pub = rospy.Publisher("Camera", Int16, queue_size=10)
        self.cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Error: Unable to open stream.")
            exit()
        self.processFeed()

    def processFeed(self):
        global counter
        global facesCounter
        global message
        global lastMessage
        global stream_error_counter
        while True:
            # Reading Frames
            ret, frame = self.cap.read()
            # if not ret or frame is None:
            #     rospy.logwarn("Failed to grab frame from the stream.")
            #     stream_error_counter += 1
            #     if stream_error_counter > 5:  # Try reconnecting after 5 failed attempts
            #         rospy.logwarn("Reconnecting to stream...")
            #         self.cap.release()
            #         self.cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
            #         stream_error_counter = 0
            #     continue
            
            # if ret is False:
            #     stream_error_counter += 1
            #     rospy.logwarn(f"Failed to read from stream. Retry {stream_error_counter}/5.")
            #     if stream_error_counter > 5:
            #         rospy.logwarn("Attempting to reconnect...")
            #         self.cap.release()
            #         self.cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
            #         stream_error_counter = 0  # Reset counter
            #         continue
            
            knownFlag = False
            unknownFlag = False
            # Resizing the frames to process faster
            # processingFrame = cv2.resize(frame, (0, 0), fx = 0.25, fy = 0.25)
            # processingFrame = cv2.cvtColor(processingFrame, cv2.COLOR_BGR2RGB)
            processingFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Processing once every 20 counters for smoother feed
            if counter % 20 == 0:
                # Getting the location and the encodings of each face found
                face_locations = face_recognition.face_locations(processingFrame)
                face_encodings = face_recognition.face_encodings(processingFrame, face_locations)

            # Loop in each found face
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # # Resize the photo to the original size
                # top *= 4
                # right *= 4
                # bottom *= 4
                # left *= 4

                # Find matches between found faces and trusted ones
                matches = face_recognition.compare_faces(knownFacesEncodings, face_encoding)
                
                # If there are trusted matches
                if True in matches:
                    # Draw a green rectangle and the name of the person
                    first_match_index = matches.index(True)
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    cv2.putText(frame, knownNames[first_match_index], (left + 40, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    unauthorised = False
                    knownFlag = True
                else:
                    # If there is no matches, draw a red rectangle
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                    cv2.putText(frame, "Unknown", (left + 40, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    unauthorised = True
                    unknownFlag = True
            
            # Display the frame and curr31.30:81/stream"  # Replace with your ESP32-CAM IP
# cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)ent data (for testing and debugging)
            cv2.imshow('Face Recognition', frame)
            rospy.loginfo(f"knownFlag = {knownFlag}")
            rospy.loginfo(f"unknownFlag = {unknownFlag}")
            rospy.loginfo(f"message = {message}")
            rospy.loginfo(f"lastmes = {lastMessage}")

            
            if knownFlag == True and unknownFlag == False: # If there is a known person
                message = 1
                if message != lastMessage: # Only send if the message is changed from the last state
                    msg = first_match_index + 1 # 1 for the first face and so on...
                    self.pub.publish(msg)
                    lastMessage = 1
                    rospy.loginfo("Hola Senior")
            elif knownFlag == False and unknownFlag == True: # If there is an unknown person
                message = 4
                if message != lastMessage: # Only send if the message is changed from the last state
                    msg = 0 # 0 for unknown face
                    self.pub.publish(msg)
                    lastMessage = 4
                    rospy.loginfo("a7a enta men")

            # Close the camera feed if 'e' is pressed
            if cv2.waitKey(1) & 0xFF == ord('e'):
                # ser.write(b'0')
                break
            
            # Increment the frames counter
            counter += 1
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    myNode = Camera()
    rospy.spin()