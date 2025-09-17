//  HumanoIDIOT Controller
/*
  -----------Notes-----------
  -> Maximum message value that can be received from Bluetooth Module is "255", DON'T EXCEED THAT!
  -> Centered orientation for the Humanoid is bodyAngle = 75, headAngle = 100, armAngle = 30
  -> Function "stopRobot" halts all motion but keeps sensors and communication running
  -> Messages
  0 --> Stop the Robot
  1 --> Turn Right (slowly)
  2 --> Turn Left (slowly)
  3 --> Move Forward (slowly)
  4 --> Move Backwards (slowly)
  5 --> Move Forward (quickly)
  6 --> Move Backward (quickly)
  7 --> Turn Right (quickly)
  8 --> Turn Left (quickly)
  27 --> Exit Automatic Modes
  28 --> Initiate Premir Automatic Mode
  29 --> Initiate Segundo Automatic Mode
  30 --> Clean
  35 --> Hola Senior Mode
  41 --> Move arm upwards
  42 --> Move arm downwards
  43 --> Turn humanoid left
  44 --> Turn Humanoid Right
  45 --> Move head upwards
  46 --> Move head downwards
  55 --> Buzz
  69 --> Unknown Person
  70,..73 --> Known Persons
*/

/*
  -----------Controller-----------
  Slow Movement --> Left Analog
  Fast Movement --> Left Analog + R1
  Arm and Body Movement --> Right Analog
  Head Movement --> Arrows
  Initiate Premir Automatic Mode --> Triangle
  Clean --> Circle
  Initiate Segundo Automatic Mode --> X
  Buzz --> Square
  Exit Automatic Modes --> L2
*/

/*-----------Pins-----------
  Body Servo 12
  Arm Servo 11
  Head Servo 10

  ENA 6 // Right
  IN1 30
  IN2 32
  IN3 34
  IN4 36
  ENB 5 // Left
  ENCAr 2   // Green Right
  ENCBr 3   // Yellow Right
  ENCAl 18  // Yellow Left
  ENCBl 19  // Green Left

  Cleaners 4
  IR 42
  Ultrasonic Echo 44
  Ultrasonic Trig 44
  Buzzer 52
*/

// Libraries
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <MAX30102.h>
#include <Wire.h>
#include <Pulse.h>

// Encoder
#define ENCAr 2   // Green Right
#define ENCBr 3   // Yellow Right
#define ENCAl 18  // Yellow Left
#define ENCBl 19  // Green Left
volatile long posir = 0, posil = 0, avg = 0;

// Motors
#define ENA 6 // Right
#define IN1 30
#define IN2 32
#define IN3 34
#define IN4 36
#define ENB 5 // Left
#define lowSpeed 150
#define highSpeed 200
#define cleaners 4
#define cleanersSpeed 100

// Servos
Servo RIGHT_ARM;
Servo HEAD;
Servo BODY;
byte bodyAngle = 75, headAngle = 100, armAngle = 30;

// IR, Ultrasonic and Buzzer
#define IR_sensor 42
byte irReading;

#define echo 44
#define trig 7
int distance, distanceR, distanceL;
volatile long duration;

#define buzzer 52

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Oximeter
MAX30102 sensor;
Pulse pulseIR;  // Create an instance of the Pulse class for IR signal processing
MAFilter bpm;  // Create an instance of the filter for calculating BPM
long lastBeat = 0;  // Store the time of the last detected beat
int beatAvg = 0;  // Store the average beats per minute (BPM)

// Communication
byte message[2];

void setup() {
  // put your setup code here, to run once:

  // Encoders
  attachInterrupt(digitalPinToInterrupt(ENCAr), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAl), readEncoder2, RISING);

  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(cleaners, OUTPUT);

  // Servos
  HEAD.attach(10);
  RIGHT_ARM.attach(11);
  BODY.attach(12);

  // Sensors
  pinMode(IR_sensor, INPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // Humanoid Orientation
  BODY.write(bodyAngle);
  RIGHT_ARM.write(armAngle);
  HEAD.write(headAngle);

  // Communication
  Serial.begin(115200);
  Serial3.begin(9600);

  //LCD
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.println("Sensor is Ready");
  lcd.print("System Initializing");
  delay(2000);

  if (!sensor.begin()) {
    Serial.println("Sensor Error Restart Required");
    displayMessage("Sensor Error", "Restart Required");
    while (1);  // Halt the program if sensor initialization fails
  }
  sensor.setup();  // Set up the sensor for use
}

void loop() {
  // put your main code here, to run repeatedly:
  irReading = digitalRead(IR_sensor); // 0 White, 1 Black

  Serial.print("posil: ");
  Serial.print(posil);
  Serial.print("\tposir: ");
  Serial.println(posir);
  //  Serial.print("\tavg: ");
  //  Serial.println(avg);
  Serial.print("IR: ");
  Serial.print(irReading);

  // Messages
  if (Serial3.available()) {
    if (!(message[0] == 41 or message[0] == 42 or message[0] == 43 or message[0] == 44 or message[0] == 45 or message[0] == 46)) {
      // If not a Servo Message
      message[1] = message[0];
    }
    message[0] = Serial3.read();
    for (int i = 1; i >= 0; i--) {
      switch (message[i]) {
        case 0:
          stopRobot();
          //          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Stopped");
          break;
        case 1:
          turnRight(lowSpeed);
          break;
        case 2:
          turnLeft(lowSpeed);
          break;
        case 3:
          moveForward(lowSpeed);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Moving Forward");
          break;
        case 4:
          moveBackwards(lowSpeed);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Moving Backwards");
          break;
        case 5:
          moveForward(highSpeed);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Moving Forward");
          break;
        case 6:
          moveBackwards(lowSpeed);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Moving Backwards");
          break;
        case 7:
          turnRight(highSpeed);
          break;
        case 8:
          turnLeft(highSpeed);
          break;
        case 28:
          autoModeUNO();
          break;
        case 29:
          // Auto mode
          autoModeDOS();
          break;
        case 30:
          // Clean
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Cleaning");
          analogWrite(cleaners, cleanersSpeed);
          break;
        case 35:
          // Hola Senior
          holaSenior();
          break;
        case 41:
          // Arm up
          RIGHT_ARM.write(90);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Arm Up");
          break;
        case 42:
          // Arm down
          RIGHT_ARM.write(30);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Arm Down");
          break;
        case 43:
          // Arm left
          if (bodyAngle < 165) {
            bodyAngle += 30;
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("Body Left");
          }
          BODY.write(bodyAngle);
          break;
        case 44:
          // Arm right
          if (bodyAngle > 15) {
            bodyAngle -= 30;
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("Body Right");
            BODY.write(bodyAngle);
          }
          else {
            BODY.write(0);
          }
          break;
        case 45:
          // Head Up
          HEAD.write(155);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Head up");
          break;
        case 46:
          // Head Down
          HEAD.write(70);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Head Down");
          break;
        case 55:
          // Buzz
          tone(buzzer, 2000);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Buzzing");
          break;
        case 69:
          // Unknown Persons
          tone(buzzer, 2000);
          delay(2000);
          break;
        case 70:
          // First Known Person
          break;
        default:
          stopRobot();
          lcd.clear();
          //          lcd.setCursor(0, 1);
          lcd.print("Stopped");
          break;
      }
    }
  }
}

// Stop Everything
void stopRobot() {
  analogWrite(ENA, 0);
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN1 , LOW);
  analogWrite(ENB, 0);
  analogWrite(cleaners, 0);
  noTone(buzzer);
}

// Encoders
void readEncoder1() {
  int b = digitalRead(ENCBr);
  if (b > 0) {
    posir++;
  } else {
    posir--;
  }
}
void readEncoder2() {
  int b = digitalRead(ENCBl);
  if (b > 0) {
    posil++;
  } else {
    posil--;
  }
}

//  Motion
void moveForward(byte motionSpeed) {
  analogWrite(ENA, motionSpeed);
  digitalWrite(IN1 , HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , HIGH);
  digitalWrite(IN4 , LOW);
  analogWrite(ENB, motionSpeed);
}
void moveBackwards(byte motionSpeed) {
  analogWrite(ENA, motionSpeed);
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , HIGH);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
  analogWrite(ENB, motionSpeed);
}
void turnRight(byte motionSpeed) {
  analogWrite(ENA, motionSpeed);
  digitalWrite(IN1 , HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
  analogWrite(ENB, motionSpeed);
}
void turnLeft(byte motionSpeed) {
  analogWrite(ENA, motionSpeed);
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , HIGH);
  digitalWrite(IN3 , HIGH);
  digitalWrite(IN4 , LOW);
  analogWrite(ENB, motionSpeed);
}

// Ultrasonic
void readUltrasonic() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
}

// Check Modes Termination
byte checkTermination() {
  if (Serial3.available()) { // Terminate if L2 is pressed
    message[0] = Serial3.read();
    if (message[0] == 27) {
      return true;
    }
  }
  else {
    return false;
  }
}

// Auto modes
void autoModeUNO() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Auto Cleaning 1");
  BODY.write(75);
  RIGHT_ARM.write(170); // Clear the way for detection
  while (true) {
    if (checkTermination() == true) {
      return;
    }
    checkIR();
    moveForward(150);
    readUltrasonic(); // Read Distance in front
    Serial.print("\tDistance: ");
    Serial.print(distance);
    Serial.print("\tposil: ");
    Serial.print(posil);
    Serial.print("\tposir: ");
    Serial.println(posir);
    if (distance < 20) { // Something in front of the robot
      stopRobot();
      BODY.write(165); // Look Left
      delay(1500);
      readUltrasonic();
      distanceL = distance;
      BODY.write(0); // Look Right
      delay(1500);
      readUltrasonic();
      distanceR = distance;
      BODY.write(75);
      // Check which side to go
      if (distanceR > distanceL && distanceR >= 40) {
        posir = 0;
        while (posir > -400) { // Right
          if (checkTermination() == true) {
            return;
          }
          Serial.println("Right");
          Serial.print("\tposir: ");
          Serial.println(posir);
          turnRight(150);
        }
      }
      else if (distanceL > distanceR && distanceL >= 40) {
        posir = 0;
        while (posir < 400) { // Left
          if (checkTermination() == true) {
            return;
          }
          turnLeft(150);
          Serial.println("Left");
          Serial.print("\tposir: ");
          Serial.println(posir);
        }
      }
      else {
        Serial.println("Wtf!");
        tone(52, 2500);
      }
    }
    if (checkTermination() == true) {
      return;
    }
  }
}
void autoModeDOS() {
  // To be Determined
  BODY.write(75);
  byte n; // Number of times to repeat the loop
  /*
    Move forward 100 cm
    turn left 90
    Move forward 100 cm
    turn left 90
    Move forward 100 cm
    turn left 90
    Move forward 80 cm
    turn left 90
    Move forward 80 cm
    turn left 90
    Move forward 60 cm
    turn left 90
    Move forward 60 cm
    turn left 90
    Move forward 40 cm
    turn left 90
    Move forward 40 cm
    turn left 90
    Move forward 20 cm
    turn left 90
  */
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Auto Cleaning 2");
  checkIR();
  posir = 0;
  for (int i = 100; i >= 20; i -= 20) {
    if (checkTermination() == true) {
      return;
    }
    if (i == 100) {
      n = 3;
    }
    else if (i == 20) {
      n = 1;
    }
    else {
      n = 2;
    }
    checkIR();
    for (int j = 0; j < n; j++) {
      posir = 0;
      while (posir < (i / (20.4 / 514.8))) { // Forward
        if (checkTermination() == true) {
          return;
        }
        checkIR();
        moveForward(150);
        readUltrasonic();
        Serial.print("\tDistance: ");
        Serial.print(distance);
        Serial.print("\tposil: ");
        Serial.print(posil);
        Serial.print("\tposir: ");
        Serial.println(posir);
      }
      posir = 0;
      while (posir < 400) { // Left
        if (checkTermination() == true) {
          return;
        }
        checkIR();
        turnLeft(150);
        readUltrasonic();
        Serial.print("\tDistance: ");
        Serial.print(distance);
        Serial.print("\tposil: ");
        Serial.print(posil);
        Serial.print("\tposir: ");
        Serial.println(posir);
      }
      checkIR();
    }
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("\tposil: ");
    Serial.print(posil);
    Serial.print("\tposir: ");
    Serial.println(posir);
  }
  stopRobot();
  delay(10000);
}

// Hola Senior
void holaSenior() {
  int prevMillis = millis();
  HEAD.write(160);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hola Senior");
  //  //  while (millis() - prevMillis > 10000) {
  //  //    if (Serial3.available()) {
  //  //      message[0] = Serial3.read();
  //  //      HEAD.write(160);
  //  //      sensor.check();
  //  //    if (!sensor.available()) {
  //  //        delay(100);
  //  //        return;
  //  //    }
  //
  while (true) {
    if (checkTermination() == true) {
      return;
    }
    Serial.print("I am in Oxi");
    sensor.check();
    if (!sensor.available()) {
      delay(100);
    }
    uint32_t irValue = sensor.getIR();  // Read the IR sensor value
    sensor.nextSample();
//    if (irValue < 5000) {  // If the IR value is too low, ask for finger placement
//      Serial.println("Place Finger To Start");
//      displayMessage("Place Finger", "To Start");
//      delay(200);
//    }

    int16_t IR_signal = pulseIR.dc_filter(irValue);  // Filter the IR signal for DC component
    bool beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));  // Check if a beat is detected

    if (beatIR) {   // If a beat is detected
      long btpm = 60000 / (millis() - lastBeat);  // Calculate beats per minute (BPM)
      if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);  // Filter the BPM value
      lastBeat = millis();  // Update the last beat time
    }

    lcd.setCursor(0, 0);
    Serial.print("Pulse Rate = ");
    Serial.println(beatAvg);
    lcd.print("Pulse Rate = ");
    lcd.print(beatAvg);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    Serial.println("Normal bpm= 60-100");
    lcd.print("bpm= 60-100");
  }
}

void checkIR() {
  if (digitalRead(IR_sensor) == 1) { // 0 White, 1 Black
    stopRobot();
    tone(buzzer, 500);
    delay(2000);
    analogWrite(cleaners, cleanersSpeed);
    delay(5000);
    analogWrite(cleaners, 0);
    noTone(buzzer);
    moveForward(150);
    delay(1500);
  }
  return;
}

void displayMessage(const String &line1, const String &line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
