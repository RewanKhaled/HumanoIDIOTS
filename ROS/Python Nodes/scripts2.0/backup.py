# #! /usr/bin/env python3
# import rospy
# from std_msgs.msg import String
# import socket

# # Set up server
# HOST = "192.168.1.13"  # Your laptop's IP address
# PORT = 12345  # Port number

# class Receiver:
#     def __init__(self):
#         rospy.init_node("Listener")
#         self.pub = rospy.Publisher("voice", String, queue_size = 10)
#         self.server_socket = self.initialize_server_socket()
#         self.publish()
    
#     def initialize_server_socket(self):
#         self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
#         self.server_socket.bind((HOST, PORT))
#         self.server_socket.listen(5)  # Allow multiple connections
#         rospy.loginfo(f"Server running on {HOST}:{PORT}")
#         return self.server_socket
    
#     def publish(self):
#         try:
#             while True:  # Continuously accept new connections
#                 rospy.loginfo("Waiting for a connection...")
#                 conn, addr = self.server_socket.accept()
#                 rospy.loginfo(f"Connected by {addr}")
                
#                 try:
#                     while True:  # Continuously receive data from the current connection
#                         data = conn.recv(1024).decode()
#                         if not data:  # Client closed connection
#                             rospy.logerr("Client disconnected")
#                             break
                        
#                         # Process and print received data
#                         if "\r\n\r\n" in data:
#                             _, body = data.split("\r\n\r\n", 1)
#                             rospy.loginfo(f"Received: {body.strip()}")
#                             break
#                         else:
#                             rospy.loginfo(f"Received: {data.strip()}")
#                             break
                
#                 except ConnectionResetError:
#                     rospy.logerr("Connection was forcibly closed by the client")

#                 except KeyboardInterrupt:
#                     rospy.logwarn("Shutting down server...")

#                 finally:
#                     conn.close()  # Ensure the connection is properly closed
#                     # Re-initialize the server socket after closing the connection
#                     self.server_socket.close()  # Close the current server socket
#                     self.server_socket = self.initialize_server_socket()  # Re-initialize the socket for the next connection

#         except KeyboardInterrupt:
#             rospy.logwarn("Shutting down server...")
            
#         finally:
#             self.server_socket.close()

# if __name__ == "__main__":
#     myNode = Receiver()
#     rospy.spin()



#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import select

# Set up server
HOST = "192.168.1.13"  # Your laptop's IP address
PORT = 12345  # Port number

class Receiver:
    def __init__(self):
        rospy.init_node("Listener")
        self.pub = rospy.Publisher("voice", String, queue_size = 10)
        self.server_socket = self.initialize_server_socket()
        self.waitForConnection()
    
    def initialize_server_socket(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(5)  # Allow multiple connections
        rospy.loginfo(f"Server running on {HOST}:{PORT}")
        return self.server_socket

    def waitForConnection(self):
        try:
            while True:  # Continuously accept new connections
                rospy.loginfo("Waiting for a connection...")
                
                self.conn, addr = self.server_socket.accept()
                rospy.loginfo(f"Connected by {addr}")
                self.publish()

        except KeyboardInterrupt:
            rospy.logwarn("Shutting down server...")

        finally:
            self.server_socket.close()

    def publish(self):
        try:
            while True:  # Continuously receive data from the current connection
                data = self.conn.recv(1024).decode()
                if not data:  # Client closed connection
                    rospy.logerr("Client disconnected")
                    break
                        
                # Process and print received data
                if "\r\n\r\n" in data:
                    _, body = data.split("\r\n\r\n", 1)
                    rospy.loginfo(f"Received: {body.strip()}")
                    break
                else:
                    rospy.loginfo(f"Received: {data.strip()}")
                    break
                
        except ConnectionResetError:
            rospy.logerr("Connection was forcibly closed by the client")

        except KeyboardInterrupt:
            rospy.logwarn("Shutting down server...")

        finally:
            self.conn.close()  # Ensure the connection is properly closed
            # Re-initialize the server socket after closing the connection
            self.server_socket.close()  # Close the current server socket
            self.server_socket = self.initialize_server_socket()  # Re-initialize the socket for the next connection

        

if __name__ == "__main__":
    myNode = Receiver()
    rospy.spin()



#! /usr/bin/env python3
import rospy
import cv2
import face_recognition
from std_msgs.msg import String

# Camera URL (Video URL not the whole server)
stream_url = "http://192.168.231.30:81/stream"

# Trusted Faces List
knownFaces = [face_recognition.load_image_file("captured_face.jpg")]
knownNames = ["MWA"] # A name for each trusted face
knownFacesEncodings = [face_recognition.face_encodings(image)[0] for image in knownFaces] # Encodings for each face

# Ignored Faces List (Initialized with no faces)
ignoredFaces = []

# Counters
counter = 0 # Frame Counter
facesCounter = len(knownFaces) + 1 #Faces Counter (Initialized with the number of known faces)
ignoredFacesCounter = 0 # Ignored Faces Counter
message = 0
lastMessage = 90
stream_error_counter = 0

class Camera:
    def __init__(self):
        self.pub = rospy.Publisher("Camera", String, queue_size=10)
        cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            rospy.logerr("Error: Unable to open stream.")
            exit()
        self.processFeed()

    def processFeed(self):
        while True:
            # Reading Frames
            ret, frame = cap.read()
            if not ret or frame is None:
                rospy.logwarn("Failed to grab frame from the stream.")
                stream_error_counter += 1
                if stream_error_counter > 5:  # Try reconnecting after 5 failed attempts
                    rospy.logwarn("Reconnecting to stream...")
                    cap.release()
                    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
                    stream_error_counter = 0
                continue
            
            if ret is False:
                stream_error_counter += 1
                rospy.logwarn(f"Failed to read from stream. Retry {stream_error_counter}/5.")
                if stream_error_counter > 5:
                    rospy.logwarn("Attempting to reconnect...")
                    cap.release()
                    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
                    stream_error_counter = 0  # Reset counter
                    continue
            
            knownFlag = False
            unknownFlag = False
            # Resizing the frames to process faster
            processingFrame = cv2.resize(frame, (0, 0), fx = 0.25, fy = 0.25)
            processingFrame = cv2.cvtColor(processingFrame, cv2.COLOR_BGR2RGB)

            # Processing once every 20 counters for smoother feed
            if counter % 20 == 0:
                # Getting the location and the encodings of each face found
                face_locations = face_recognition.face_locations(processingFrame)
                face_encodings = face_recognition.face_encodings(processingFrame, face_locations)

            # Flag of unauthorised persons
            unauthorised = False

            # Loop in each found face
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # Resize the photo to the original size
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                # Find matches between found faces and trusted ones
                matches = face_recognition.compare_faces(knownFacesEncodings, face_encoding)
                if ignoredFacesCounter > 0:
                    # If there is an Ignored Face, # Find matches between found faces and ignored ones
                    ignoredMatches = face_recognition.compare_faces(ignoredFacesEncodings, face_encoding)

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
                    # Check if the person is already ignored
                    if ignoredFacesCounter > 0 and True in ignoredMatches:
                        unauthorised = False
            
            # Display the frame and current data (for testing and debugging)
            cv2.imshow('Face Recognition', frame)
            rospy.loginfo(f"knownFlag = {knownFlag}")
            rospy.loginfo(f"unknownFlag = {unknownFlag}")
            rospy.loginfo(f"message = {message}")
            rospy.loginfo(f"lastmes = {lastMessage}")

            # If there is an unknown face (Neither trusted nor ignored)
            if unauthorised == True and counter % 20 == 0:
                # Display a message to the house residents and prompt them for an action
                rospy.logwarn("Unauthorised Person at the door! add or ignore?")
                command = input("Add/Open/Ignore ")
                unauthorised == False
                if command == "Add":
                    facesCounter += 1
                    # Capture the face of the person and ask for his name
                    newName = input("Name? ")
                    face_roi = frame[top:bottom, left:right]
                    # Save the image of the face
                    cv2.imwrite(f"captured_face{facesCounter}.jpg", face_roi)
                    rospy.log("Photo captured!")
                    # Add the face to the trusted list
                    knownFaces.append(face_recognition.load_image_file(f"captured_face{facesCounter}.jpg"))
                    knownNames.append(newName)
                    knownFacesEncodings = [face_recognition.face_encodings(image)[0] for image in knownFaces]
                    counter += 1
                    continue
                elif command == "Ignore" or command == "Open":
                    ignoredFacesCounter += 1
                    # Capture the face of the person
                    face_roi = frame[top:bottom, left:right]
                    # Save the image of the face
                    cv2.imwrite(f"ignored_face{ignoredFacesCounter}.jpg", face_roi)
                    rospy.log("Photo captured!")
                    # Add the face to the ignored list
                    ignoredFaces.append(face_recognition.load_image_file(f"ignored_face{ignoredFacesCounter}.jpg"))
                    ignoredFacesEncodings = [face_recognition.face_encodings(image)[0] for image in ignoredFaces]
                    if command == "Open":
                        #Send signal to the door
                        rospy.log("door opening")
                        message = 2
                        if message != lastMessage: # Only send if the message is changed from the last state
                            # ser.write(b"2")
                            lastMessage = 2
                    elif command == "Ignore":
                        rospy.log("Ignored")
                        message = 3
                        if message != lastMessage: # Only send if the message is changed from the last state
                            # ser.write(b"3")
                            lastMessage = 3
                    counter += 1
                    continue
            
            if knownFlag == True and unknownFlag == False: # If there is a known person
                message = 1
                if message != lastMessage: # Only send if the message is changed from the last state
                    # ser.write(b"1")
                    lastMessage = 1
            elif knownFlag == False and unknownFlag == True: # If there is an unknown person
                message = 4
                if message != lastMessage: # Only send if the message is changed from the last state
                    # ser.write(b"4")
                    lastMessage = 4

            # Close the camera feed if 'e' is pressed
            if cv2.waitKey(1) & 0xFF == ord('e'):
                # ser.write(b'0')
                break
            
            # Increment the frames counter
            counter += 1
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    myNode = Camera()
    rospy.spin()



# #! /usr/bin/env python3
# import rospy
# import cv2
# import face_recognition
# from std_msgs.msg import String

# # Camera URL (Video URL not the whole server)
# stream_url = "http://192.168.231.30:81/stream"

# # Trusted Faces List
# knownFaces = [face_recognition.load_image_file("captured_face.jpg")]
# knownNames = ["MWA"] # A name for each trusted face
# knownFacesEncodings = [face_recognition.face_encodings(image)[0] for image in knownFaces] # Encodings for each face

# # Ignored Faces List (Initialized with no faces)
# ignoredFaces = []

# # Counters
# counter = 0 # Frame Counter
# facesCounter = len(knownFaces) + 1 #Faces Counter (Initialized with the number of known faces)
# ignoredFacesCounter = 0 # Ignored Faces Counter
# message = 0
# lastMessage = 90
# stream_error_counter = 0

# class Camera:
#     def __init__(self):
#         self.pub = rospy.Publisher("Camera", String, queue_size=10)
#         cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
#         cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#         if not cap.isOpened():
#             rospy.logerr("Error: Unable to open stream.")
#             exit()
#         self.processFeed()

#     def processFeed(self):
#         while True:
#             # Reading Frames
#             ret, frame = cap.read()
#             if not ret or frame is None:
#                 rospy.logwarn("Failed to grab frame from the stream.")
#                 stream_error_counter += 1
#                 if stream_error_counter > 5:  # Try reconnecting after 5 failed attempts
#                     rospy.logwarn("Reconnecting to stream...")
#                     cap.release()
#                     cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
#                     stream_error_counter = 0
#                 continue
            
#             if ret is False:
#                 stream_error_counter += 1
#                 rospy.logwarn(f"Failed to read from stream. Retry {stream_error_counter}/5.")
#                 if stream_error_counter > 5:
#                     rospy.logwarn("Attempting to reconnect...")
#                     cap.release()
#                     cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
#                     stream_error_counter = 0  # Reset counter
#                     continue
            
#             knownFlag = False
#             unknownFlag = False
#             # Resizing the frames to process faster
#             processingFrame = cv2.resize(frame, (0, 0), fx = 0.25, fy = 0.25)
#             processingFrame = cv2.cvtColor(processingFrame, cv2.COLOR_BGR2RGB)

#             # Processing once every 20 counters for smoother feed
#             if counter % 20 == 0:
#                 # Getting the location and the encodings of each face found
#                 face_locations = face_recognition.face_locations(processingFrame)
#                 face_encodings = face_recognition.face_encodings(processingFrame, face_locations)

#             # Flag of unauthorised persons
#             unauthorised = False

#             # Loop in each found face
#             for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
#                 # Resize the photo to the original size
#                 top *= 4
#                 right *= 4
#                 bottom *= 4
#                 left *= 4

#                 # Find matches between found faces and trusted ones
#                 matches = face_recognition.compare_faces(knownFacesEncodings, face_encoding)
#                 if ignoredFacesCounter > 0:
#                     # If there is an Ignored Face, # Find matches between found faces and ignored ones
#                     ignoredMatches = face_recognition.compare_faces(ignoredFacesEncodings, face_encoding)

#                 # If there are trusted matches
#                 if True in matches:
#                     # Draw a green rectangle and the name of the person
#                     first_match_index = matches.index(True)
#                     cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
#                     cv2.putText(frame, knownNames[first_match_index], (left + 40, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
#                     unauthorised = False
#                     knownFlag = True
#                 else:
#                     # If there is no matches, draw a red rectangle
#                     cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
#                     cv2.putText(frame, "Unknown", (left + 40, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
#                     unauthorised = True
#                     unknownFlag = True
#                     # Check if the person is already ignored
#                     if ignoredFacesCounter > 0 and True in ignoredMatches:
#                         unauthorised = False
            
#             # Display the frame and current data (for testing and debugging)
#             cv2.imshow('Face Recognition', frame)
#             rospy.loginfo(f"knownFlag = {knownFlag}")
#             rospy.loginfo(f"unknownFlag = {unknownFlag}")
#             rospy.loginfo(f"message = {message}")
#             rospy.loginfo(f"lastmes = {lastMessage}")

#             # If there is an unknown face (Neither trusted nor ignored)
#             if unauthorised == True and counter % 20 == 0:
#                 # Display a message to the house residents and prompt them for an action
#                 rospy.logwarn("Unauthorised Person at the door! add or ignore?")
#                 command = input("Add/Open/Ignore ")
#                 unauthorised == False
#                 if command == "Add":
#                     facesCounter += 1
#                     # Capture the face of the person and ask for his name
#                     newName = input("Name? ")
#                     face_roi = frame[top:bottom, left:right]
#                     # Save the image of the face
#                     cv2.imwrite(f"captured_face{facesCounter}.jpg", face_roi)
#                     rospy.log("Photo captured!")
#                     # Add the face to the trusted list
#                     knownFaces.append(face_recognition.load_image_file(f"captured_face{facesCounter}.jpg"))
#                     knownNames.append(newName)
#                     knownFacesEncodings = [face_recognition.face_encodings(image)[0] for image in knownFaces]
#                     counter += 1
#                     continue
#                 elif command == "Ignore" or command == "Open":
#                     ignoredFacesCounter += 1
#                     # Capture the face of the person
#                     face_roi = frame[top:bottom, left:right]
#                     # Save the image of the face
#                     cv2.imwrite(f"ignored_face{ignoredFacesCounter}.jpg", face_roi)
#                     rospy.log("Photo captured!")
#                     # Add the face to the ignored list
#                     ignoredFaces.append(face_recognition.load_image_file(f"ignored_face{ignoredFacesCounter}.jpg"))
#                     ignoredFacesEncodings = [face_recognition.face_encodings(image)[0] for image in ignoredFaces]
#                     if command == "Open":
#                         #Send signal to the door
#                         rospy.log("door opening")
#                         message = 2
#                         if message != lastMessage: # Only send if the message is changed from the last state
#                             # ser.write(b"2")
#                             lastMessage = 2
#                     elif command == "Ignore":
#                         rospy.log("Ignored")
#                         message = 3
#                         if message != lastMessage: # Only send if the message is changed from the last state
#                             # ser.write(b"3")
#                             lastMessage = 3
#                     counter += 1
#                     continue
            
#             if knownFlag == True and unknownFlag == False: # If there is a known person
#                 message = 1
#                 if message != lastMessage: # Only send if the message is changed from the last state
#                     # ser.write(b"1")
#                     lastMessage = 1
#             elif knownFlag == False and unknownFlag == True: # If there is an unknown person
#                 message = 4
#                 if message != lastMessage: # Only send if the message is changed from the last state
#                     # ser.write(b"4")
#                     lastMessage = 4

#             # Close the camera feed if 'e' is pressed
#             if cv2.waitKey(1) & 0xFF == ord('e'):
#                 # ser.write(b'0')
#                 break
            
#             # Increment the frames counter
#             counter += 1
#         cap.release()
#         cv2.destroyAllWindows()


# if __name__ == "__main__":
#     myNode = Camera()
#     rospy.spin()