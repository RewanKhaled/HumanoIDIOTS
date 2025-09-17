import serial
import time

# Replace 'COM3' with the actual COM port your laptop is using
PORT = '/dev/rfcomm0'  # For Windows, it could be 'COM4', etc.
BAUD_RATE = 9600  # Match this with the baud rate used by the Bluetooth device
while True:
    try:
        # Open the serial port
        bluetooth_serial = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {PORT}")
        time.sleep(5)
        # while True:
        #     # Read data from the serial port
        if bluetooth_serial.in_waiting > 0:
            data = bluetooth_serial.readline()
            print(f"Received: {data}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
            bluetooth_serial.close()
            print("Serial connection closed.")