//  HumanoIDIOT Controller
/*
  -----------Notes-----------
-> Maximum message value that can be received from Bluetooth Module is "255", DON'T EXCEED THAT!
-> Centered orientation for the Humanoid is bodyAngle = 75, headAngle = 45, armAngle = 30
-> Function "stopRobot" halts all motion but keeps sensors and communication running 
-> Messages:
0 --> Stop the Robot
1 --> Turn Right (slowly)
2 --> Turn Left (slowly)
3 --> Move Forward (slowly)
4 --> Move Backwards (slowly)
5 --> Move Forward (quickly)
6 --> Move Backward (quickly)
7 --> Turn Right (quickly)
8 --> Turn Left (quickly)
29 --> Initiate Automatic Mode
41 --> Move arm upwards
42 --> Move arm downwards
43 --> Turn humanoid left
44 --> Turn Humanoid Right
45 --> Move head upwards
46 --> Move head downwards
69 --> Unknown Person
70,..73 --> Known Persons
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
Obstacle 44
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
#define cleanersSpeed 50

// Servos
Servo RIGHT_ARM;
Servo HEAD;
Servo BODY;
byte bodyAngle = 75, headAngle = 45, armAngle = 30;

// IR, Obstacle and Buzzer
#define IR_sensor 42
byte irReading;
#define Obstacle 44
byte obstacleReading;
#define buzzer 52

// Communication
byte message;

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
  pinMode(Obstacle, INPUT);

  // Outputs
  pinMode(buzzer, OUTPUT);
  
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
  obstacleReading = digitalRead(Obstacle); // 0 --> Something, 1 --> Nothing
  Serial.print("posil: ");
  Serial.print(posil);
  Serial.print("\tposir: ");
  Serial.println(posir);
//  Serial.print("\tavg: ");
//  Serial.println(avg);
  Serial.print("IR: ");
  Serial.print(irReading);
  Serial.print("   Obstacle: ");
  Serial.println(obstacleReading);

  // Messages
  if (Serial3.available()) {
    message = Serial3.read();
    switch (message) {
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
      case 29:
        // Auto mode
        autoMode();
        break;
      case 30:
        // Clean
        analogWrite(cleaners, cleanersSpeed);
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
        if (bodyAngle < 165){
          bodyAngle += 30;
        }
        BODY.write(bodyAngle);
        break;
      case 44:
        // Arm right
        if (bodyAngle > 15){
          bodyAngle -= 30;
        }
        BODY.write(bodyAngle);
        break;
      case 45:
        // Head Up
//        headAngle -= 60;
        HEAD.write(90);
        break;
      case 46:
        // Head Down
//        headAngle -= 60;
        HEAD.write(45);
        break;
      case 69:
        // Unknown Person
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

// Stop Everything
void stopRobot() {
  analogWrite(ENA, 0);
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN1 , LOW);
  analogWrite(ENA, 0);
  analogWrite(cleaners, 50);
}

// Encoders
void readEncoder1() {
  int b = digitalRead(ENCBr);
  if (b > 0) {
    posir--;
  } else {
    posir++;
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
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , HIGH);
  digitalWrite(IN3 , HIGH);
  digitalWrite(IN4 , LOW);
  analogWrite(ENB, motionSpeed);
}
void turnLeft(byte motionSpeed) {
  analogWrite(ENA, motionSpeed);
  digitalWrite(IN1 , HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
  analogWrite(ENB, motionSpeed);
}

// Auto mode
void autoMode(){
// To be Determined
}
