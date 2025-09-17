# import socket

# # Set up server
# HOST = "192.168.1.13"  # Get the laptop's IP
# PORT = 12345  # Use the same port as in the MIT App Inventor app

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind((HOST, PORT))
# server_socket.listen(1)

# print(f"Server running on {HOST}:{PORT}")

# # Accept a connection
# conn, addr = server_socket.accept()
# print(f"Connected by {addr}")

# while True:
#     data = conn.recv(1024).decode()
#     if not data:
#         break
#     print(f"Received: {data}")

# conn.close()
# server_socket.close()

# import socket

# # Set up server
# HOST = "192.168.1.13"  # Your laptop's IP address
# PORT = 12345  # Port number

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# server_socket.bind((HOST, PORT))
# server_socket.listen(5)

# print(f"Server running on {HOST}:{PORT}")

# # Accept a connection
# conn, addr = server_socket.accept()
# print(f"Connected by {addr}")

# try:
#     while True:
#         data = conn.recv(1024).decode()  # Receive the data
#         if not data:  # No data means the connection is closed
#             print("Connection closed by client")
#             break
        
#         # Parse the body of the HTTP request
#         if "\r\n\r\n" in data:  # Separate headers and body
#             _, body = data.split("\r\n\r\n", 1)  # Split at the empty line
#             print(f"Received: {body.strip()}")  # Print only the body
#         else:
#             print("Incomplete HTTP request received")

# finally:
#     conn.close()
#     server_socket.close()
import socket

# Function to set up the server socket
def initialize_server_socket():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)  # Allow multiple connections
    print(f"Server running on {HOST}:{PORT}")
    return server_socket

# Set up server
HOST = "192.168.1.13"  # Your laptop's IP address
PORT = 12345  # Port number

server_socket = initialize_server_socket()  # Initialize the server socket

try:
    while True:  # Continuously accept new connections
        print("Waiting for a connection...")
        conn, addr = server_socket.accept()
        print(f"Connected by {addr}")
        
        try:
            while True:  # Continuously receive data from the current connection
                data = conn.recv(1024).decode()
                if not data:  # Client closed connection
                    print("Client disconnected")
                    break
                
                # Process and print received data
                if "\r\n\r\n" in data:
                    _, body = data.split("\r\n\r\n", 1)
                    print(f"Received: {body.strip()}")
                    break
                else:
                    print(f"Received: {data.strip()}")
                    break
        
        except ConnectionResetError:
            print("Connection was forcibly closed by the client")
        
        finally:
            conn.close()  # Ensure the connection is properly closed
            # Re-initialize the server socket after closing the connection
            server_socket.close()  # Close the current server socket
            server_socket = initialize_server_socket()  # Re-initialize the socket for the next connection

except KeyboardInterrupt:
    print("Shutting down server...")
finally:
    server_socket.close()
