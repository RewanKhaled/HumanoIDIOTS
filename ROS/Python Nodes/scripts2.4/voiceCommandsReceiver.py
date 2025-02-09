#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import select
import sys

# Set up server
# HOST = "192.168.235.84"  # laptop's IP address
# HOST = "192.168.1.13"  # laptop's IP address
HOST = "192.168.124.84"
PORT = 12345  # Port number
# Connect from the mobile using http://192.168.121.84:12345

class Receiver:
    def __init__(self):
        rospy.init_node("Listener") # Initialize Node
        self.pub = rospy.Publisher("voice", String, queue_size=10) # Publisher
        self.server_socket = self.initialize_server_socket() # Initialize connection with server

    def initialize_server_socket(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(5)
        rospy.loginfo(f"Server running on {HOST}:{PORT}")
        return server_socket

    def waitForConnection(self):
        try:
                rospy.loginfo("Waiting for a connection...")
                read, _, _ = select.select([self.server_socket, sys.stdin], [], [], 300.0)  # Timeout of 60 seconds
                # After 60 seconds of no usage, the Node stops listening and is ready to be terminated
                rospy.loginfo(read)
                for sock in read:
                    if sock == self.server_socket: # Read from the server
                        self.conn, addr = self.server_socket.accept()
                        rospy.loginfo(f"Connected by {addr}")
                        self.publish()
                    elif sock == sys.stdin: # Terminate
                        rospy.logwarn("Keyboard input detected, terminating...")
                        raise KeyboardInterrupt()
                        
        except KeyboardInterrupt:
            rospy.logwarn("Shutting down server...")

        finally:
            self.server_socket.close()  # Ensure the server socket is closed

    def publish(self):
        try:
            while not rospy.is_shutdown():  # Continuously receive data from the current connection
                data = self.conn.recv(1024).decode()
                if not data:  # Client closed connection
                    rospy.logerr("Client disconnected")
                    break

                # Process and print received data
                if "\r\n\r\n" in data:
                    _, body = data.split("\r\n\r\n", 1)
                    rospy.loginfo(f"Received: {body.strip()}")
                    self.pub.publish(body.strip())  # Publish received msg only
                    break  # Exit after processing the data
                else:
                    rospy.loginfo(f"Received: {data.strip()}")
                    self.pub.publish(data.strip())  # Publish received msg only
                    break  # Exit after processing the data

        except ConnectionResetError:
            rospy.logerr("Connection was forcibly closed by the client")
        except KeyboardInterrupt:
            rospy.logwarn("Shutting down server...")

        finally:
            self.conn.close()  # Ensure the connection is properly closed
            self.server_socket.close()  # Close the current server socket
            # Re-initialize the server socket after closing the connection
            self.server_socket = self.initialize_server_socket()
            self.waitForConnection() # Wait for data

if __name__ == "__main__":
    myNode = Receiver()
    myNode.waitForConnection()  # Wait for data connections
    rospy.spin()
