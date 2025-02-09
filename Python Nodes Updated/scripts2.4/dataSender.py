#! /usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String, Int16
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
        rospy.Subscriber("Camera", Int16, self.cameraCallback) # Subscribe to voice
        # Initialize Variables
        self.newMsg = 0
        self.motionArr = [0, 0, 0, 0, 0, 0]
        self.buttonsArr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.voice = ""

    def joyCallback(self, msg:Joy): # Receive Joystick data
        self.motionArr = msg.axes
        self.buttonsArr = msg.buttons
        self.voice = "" # Terminate Voice Command by Joystick
        self.send()
        
    def voiceCallback(self, msg:String): # Receive Voice data
        self.voice = msg.data
        self.send()

    def cameraCallback(self, msg:Int16):
        face = msg.data # 0 for unknown, 1, 2, 3,.. for known
        rospy.loginfo(face + 69)
        ser.write(bytes([face + 69]))

    def send(self): # Send to Arduino
        if self.motionArr[0] == 1 and self.buttonsArr[5] == 1: # Fast Left
            self.newMsg = 8
        elif self.motionArr[0] == -1 and self.buttonsArr[5] == 1 : # Fast Right
            self.newMsg = 7
        elif self.motionArr[1] == 1 and self.buttonsArr[5] == 1: # Fast Forward
            self.newMsg = 5
        elif self.motionArr[1] == -1 and self.buttonsArr[5] == 1: # Fast Backward
            self.newMsg = 6
        elif self.motionArr[2] == 1:
            self.newMsg = 41 # UP Arm
        elif self.motionArr[2] == -1:
            self.newMsg = 42 # DOWN Arm
        elif self.motionArr[3] == 1:
            self.newMsg = 43 # Left Body
        elif self.motionArr[3] == -1:
            self.newMsg = 44 # Right Body
        elif self.motionArr[5] == 1:
            self.newMsg = 45 # Head Up
        elif self.motionArr[5] == -1:
            self.newMsg = 46 # Head Down
        elif self.motionArr[0] == 1 or self.voice == "turn left": # Left
            self.newMsg = 2
        elif self.motionArr[0] == -1 or self.voice == "turn right": # Right
            self.newMsg = 1
        elif self.motionArr[1] == 1 or self.voice == "move forward": # Forward
            self.newMsg = 3
        elif self.motionArr[1] == -1 or self.voice == "move backwards": # Backward
            self.newMsg = 4
        elif self.buttonsArr[6] == 1: # Interrupt Auto Modes
            self.newMsg = 27
        elif self.buttonsArr[0] == 1: # Auto Mode Uno
            self.newMsg = 28
        elif self.buttonsArr[2] == 1: # Auto Mode Dos
            self.newMsg = 29
        elif self.buttonsArr[1] == 1 or self.voice == "clean": # Clean
            self.newMsg = 30
        elif self.buttonsArr[7] == 1 or self.voice == "hello":
            self.newMsg = 35
        elif self.motionArr[2] == 1:
            self.newMsg = 41 # UP Arm
        elif self.motionArr[2] == -1:
            self.newMsg = 42 # DOWN Arm
        elif self.motionArr[3] == 1:
            self.newMsg = 43 # Left Body
        elif self.motionArr[3] == -1:
            self.newMsg = 44 # Right Body
        elif self.motionArr[5] == 1:
            self.newMsg = 45 # Head Up
        elif self.motionArr[5] == -1:
            self.newMsg = 46 # Head Down
        elif self.buttonsArr[3] == 1:
            self.newMsg = 55 # Buzz
        else:
            self.newMsg = 0 # Stop
        rospy.loginfo(self.newMsg)
        ser.write(bytes([self.newMsg]))



if __name__ == "__main__":
    myNode = Control()
    rospy.spin()