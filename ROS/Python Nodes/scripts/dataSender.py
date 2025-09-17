#! /usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# Initialize the serial connection to the Bluetooth module
bluetooth_port = '/dev/rfcomm2'  # Replace with your Bluetooth serial port
baud_rate = 9600  # Adjust based on your module configuration
ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)

class Control:
    def __init__(self):
        rospy.init_node("IdiotControl") # Initialize the Node
        rospy.Subscriber("joy", Joy, self.joyCallback) # Subscribe to joy
        rospy.Subscriber("voice", String, self.voiceCallback) # Subscribe to voice
        # Initialize Variables
        self.newMsg = 0
        self.motionArr = [0, 0, 0, 0, 0, 0]
        self.buttonsArr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.voice = ""

    def joyCallback(self, msg:Joy): # Receive Joystick data
        self.motionArr = msg.axes
        self.buttonsArr = msg.buttons
        self.send()
        
    def voiceCallback(self, msg:String): # Receive Voice data
        self.voice = msg.data
        self.send()

    def send(self): # Send to Arduino
        if self.motionArr[0] == 1 or self.voice == "turn left": # Left
            self.newMsg = 2
        elif self.motionArr[0] == -1 or self.voice == "turn right": # Right
            self.newMsg = 1
        elif self.motionArr[1] == 1 or self.voice == "move forward": # Forward
            self.newMsg = 3
        elif self.motionArr[1] == -1 or self.voice == "move backward": # Backward
            self.newMsg = 4
        elif self.motionArr[0] == 1 and self.buttonsArr[5] == 1: # Fast Left
            self.newMsg = 8
        elif self.motionArr[0] == -1 and self.buttonsArr[5] == 1 : # Fast Right
            self.newMsg = 7
        elif self.motionArr[1] == 1 and self.buttonsArr[5] == 1: # Fast Forward
            self.newMsg = 5
        elif self.motionArr[1] == -1 and self.buttonsArr[5] == 1: # Fast Backward
            self.newMsg = 6
        elif self.buttonsArr[6] == 1: # Auto Mode
            self.newMsg = 29
        else:
            self.newMsg = 0 # Stop
        rospy.loginfo(self.newMsg)
        ser.write(bytes([self.newMsg]))



if __name__ == "__main__":
    myNode = Control()
    rospy.spin()