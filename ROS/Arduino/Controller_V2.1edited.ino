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
#include <Pulse.h>
#include <MAX30102.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/sleep.h>

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

// Communication
byte message[2];


// LCD and Oximeter
LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

long lastBeat = 0;
int beatAvg = 0, SPO2 = 0, SPO2f = 0;
uint8_t istate = 0, sleep_counter = 0;

const uint8_t spo2_table[184] PROGMEM = {
  100, 99, 98, 97, 97, 96, 95, 95, 94, 93, 92, 92, 91, 90, 89, 89,
  88, 87, 86, 86, 85, 84, 83, 83, 82, 81, 80, 80, 79, 78, 77, 77,
  76, 75, 74, 74, 73, 72, 71, 71, 70, 69, 68, 68, 67, 66, 65, 65,
  64, 63, 62, 62, 61, 60, 59, 59, 58, 57, 56, 56, 55, 54, 53, 53,
  52, 51, 50, 50, 49, 48, 47, 47, 46, 45, 44, 44, 43, 42, 41, 41,
  40, 39, 38, 38, 37, 36, 35, 35, 34, 33, 32, 32, 31, 30, 29, 29,
  28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 20, 20, 19, 18, 17, 17,
  16, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 5,
  4, 3, 2, 2, 1, 0
};
void displayMessage(const String &line1, const String &line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

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

  // Outputs
  //  pinMode(buzzer, OUTPUT);

  // Humanoid Orientation
  BODY.write(bodyAngle);
  RIGHT_ARM.write(armAngle);
  HEAD.write(headAngle);

  // Communication
  Serial.begin(115200);
  Serial3.begin(9600);

  //LCD
  /*lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.println("Sensor is Ready");
  lcd.print("Sensor is Ready");
  delay(2000);

  if (!sensor.begin()) {
    Serial.println("Sensor Error Restart Required");
    displayMessage("Sensor Error", "Restart Required");
    while (1);
  }
  sensor.setup();*/

  
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
      message[1] = message[0];
    }
    message[0] = Serial3.read();
    for (int i = 1; i >= 0; i--) {
      switch (message[i]) {
        case 0:
          stopRobot();
          break;
        case 1:
          turnRight(lowSpeed);
          break;
        case 2:
          turnLeft(lowSpeed);
          break;
        case 3:
          moveForward(lowSpeed);
          break;
        case 4:
          moveBackwards(lowSpeed);
          break;
        case 5:
          moveForward(highSpeed);
          break;
        case 6:
          moveBackwards(lowSpeed);
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
          analogWrite(cleaners, cleanersSpeed);
          break;
        case 35:
          // Hola Senior
          holaSenior();
          break;
        case 41:
          // Arm up
          //        armAngle += 60;
          RIGHT_ARM.write(90);
          break;
        case 42:
          // Arm down
          //        armAngle -= 60;
          RIGHT_ARM.write(30);
          break;
        case 43:
          // Arm left
          if (bodyAngle < 165) {
            bodyAngle += 30;
          }
          BODY.write(bodyAngle);
          break;
        case 44:
          // Arm right
          if (bodyAngle > 15) {
            bodyAngle -= 30;
            BODY.write(bodyAngle);
          }
          else {
            BODY.write(0);
          }
          break;
        case 45:
          // Head Up
          //        headAngle -= 60;
          HEAD.write(140);
          break;
        case 46:
          // Head Down
          //        headAngle -= 60;
          HEAD.write(70);
          break;
        case 55:
          // Buzz
          tone(buzzer, 2000);
          break;
        case 69:
          // Unknown Person
          tone(buzzer, 2000);
          delay(2000);
          break;
        case 70:
          // First Known Person
          break;
        default:
          stopRobot();
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

// Auto modes
void autoModeUNO() {
  RIGHT_ARM.write(170);
  while (true) {
    if (Serial3.available()) {
      message[0] = Serial3.read();
      if (message[0] == 27) {
        return;
      }
    }
    moveForward(150);
    ultra5ra();
    Serial.print("I am in Loop");
    Serial.print("\tDistance: ");
    Serial.print(distance);
    Serial.print("\tposil: ");
    Serial.print(posil);
    Serial.print("\tposir: ");
    Serial.println(posir);
    if (distance < 20) {
      stopRobot();
      BODY.write(165);
      delay(1500);
      ultra5ra();
      distanceL = distance;
      BODY.write(0);
      delay(1500);
      ultra5ra();
      distanceR = distance;
      BODY.write(75);
      if (distanceR > distanceL && distanceR >= 40) {
        posir = 0;
        while (posir > -400) { // Right
          if (Serial3.available()) {
            message[0] = Serial3.read();
            if (message[0] == 27) {
              return;
            }
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
          if (Serial3.available()) {
            message[0] = Serial3.read();
            if (message[0] == 27) {
              return;
            }
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
    if (Serial3.available()) {
      message[0] = Serial3.read();
      if (message[0] == 27) {
        return;
      }
    }
  }
}
void autoModeDOS() {
  byte n;
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
  posir = 0;
  for (int i = 100; i >= 20; i -= 20) {
    if (Serial3.available()) {
      message[0] = Serial3.read();
      if (message[0] == 27) {
        return;
      }
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
    for (int j = 0; j < n; j++) {
      posir = 0;
      while (posir < (i / (20.4 / 514.8))) { // Forward
        if (Serial3.available()) {
          message[0] = Serial3.read();
          if (message[0] == 27) {
            return;
          }
        }
        moveForward(150);
        ultra5ra();
        Serial.print("I am in Forward");
        Serial.print("\tDistance: ");
        Serial.print(distance);
        Serial.print("\tposil: ");
        Serial.print(posil);
        Serial.print("\tposir: ");
        Serial.println(posir);
      }
      posir = 0;
      while (posir < 400) { // Right
        if (Serial3.available()) {
          message[0] = Serial3.read();
          if (message[0] == 27) {
            return;
          }
        }
        turnLeft(150);
        ultra5ra();
        Serial.print("I am in Right");
        Serial.print("\tDistance: ");
        Serial.print(distance);
        Serial.print("\tposil: ");
        Serial.print(posil);
        Serial.print("\tposir: ");
        Serial.println(posir);
      }
    }
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("I am in Loop");
    Serial.print("\tDistance: ");
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
  //  while (millis() - prevMillis > 10000) {
  //    if (Serial3.available()) {
  //      message[0] = Serial3.read();
  //      HEAD.write(160);
  //      sensor.check();
  //    if (!sensor.available()) {
  //        delay(100);
  //        return;
  //    }

  while (true) {
     Serial.print("I am in Oxi");
     lcd.begin(16, 2);
     lcd.init();
     lcd.backlight();
     lcd.clear();
     Serial.println("Sensor is Ready");
     lcd.print("Sensor is Ready");
     delay(2000);

  if (!sensor.begin()) {
    Serial.println("Sensor Error Restart Required");
    displayMessage("Sensor Error", "Restart Required");
    while (1);
  }
  sensor.setup();

    sensor.check();
    if (!sensor.available()) {
        delay(100); 
        return;
    }
    uint32_t irValue = sensor.getIR();
    uint32_t redValue = sensor.getRed();
    sensor.nextSample();

    if (irValue < 5000) {
      Serial.println("Place Finger To Start");
      displayMessage("Place Finger", "To Start");
      delay(200);
      //      return;
    }

    int16_t IR_signal = pulseIR.dc_filter(irValue);
    bool beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));

    if (beatIR) {
      long btpm = 60000 / (millis() - lastBeat);
      if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
      lastBeat = millis();

      long numerator   = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
      long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    Serial.print("Pulse Rate = ");
    Serial.println(beatAvg);
    lcd.print("Pulse Rate = ");
    lcd.print(beatAvg);
    lcd.setCursor(0, 1);
    Serial.println("Normal bpm= 60-100");
    lcd.print("bpm= 60-100");
    //      if (message[0] == 69) {
    //        RIGHT_ARM.write(90);
    //        tone(buzzer, 2000);
    //        //LCD print "enta meen yad"
    //      }
    //      else if (message[0] == 70) {
    //        noTone(buzzer);
    //        HEAD.write(100);
    //        //LCD print "Hello MWA"
    //      }
    //      else if (message[0] == 35) {
    //        return;
    //      }
    //      else {
    //        break;
    //      }
    //    }
    if (Serial3.available()) {
      message[0] = Serial3.read();
      if (message[0] == 27) {
        return;
      }
    }
  }
}

// Ultrasonic
void ultra5ra() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
}
