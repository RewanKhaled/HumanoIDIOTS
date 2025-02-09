#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
String state = "";  // Initialize state to avoid accidental empty strings

void setup() {
  SerialBT.begin("ESP32-Bluetooth"); // Start Bluetooth
  pinMode(2, OUTPUT);           // Set pin 2 as OUTPUT
}

void loop() {
  while (SerialBT.available()) {
    delay(10);
    char c = SerialBT.read();    // Read each character
    state += c;                  // Build the string ("turn on" or "turn off")
  }

  if (state.length() > 0) {
    Serial.println(state);       // Print the received command for debugging

    // Check if the command is "turn on"
    if (state == "turn on") {
      digitalWrite(2, HIGH);     // Turn on the device connected to pin 2
    }
    // Check if the command is "turn off"
    else if (state == "turn off") {
      digitalWrite(2, LOW);      // Turn off the device connected to pin 2
    }
    // Reset state after action is completed
    state = "";
  }
}
