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
          RIGHT_ARM.write(90);
          break;
        case 42:
          // Arm down
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
          HEAD.write(155);
          break;
        case 46:
          // Head Down
          HEAD.write(70);
          break;
        case 55:
          // Buzz
          tone(buzzer, 2000);
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
void checkTermination() {
  if (Serial3.available()) { // Terminate if L2 is pressed
    message[0] = Serial3.read();
    if (message[0] == 27) {
      return;
    }
  }
}

// Auto modes
void autoModeUNO() {
  RIGHT_ARM.write(170); // Clear the way for detection
  while (true) {
    checkTermination();
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
          checkTermination();
          Serial.println("Right");
          Serial.print("\tposir: ");
          Serial.println(posir);
          turnRight(150);
        }
      }
      else if (distanceL > distanceR && distanceL >= 40) {
        posir = 0;
        while (posir < 400) { // Left
          checkTermination();
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
    checkTermination();
  }
}
void autoModeDOS() {
  // To be Determined
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
  posir = 0;
  for (int i = 100; i >= 20; i -= 20) {
    checkTermination();
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
        checkTermination();
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
        checkTermination();
        turnLeft(150);
        readUltrasonic();
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
  //  int prevMillis = millis();
  //  while (millis() - prevMillis > 10000) {
  //    if (Serial3.available()) {
  //      message[0] = Serial3.read();
  //      HEAD.write(160);
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
  //  }
}
