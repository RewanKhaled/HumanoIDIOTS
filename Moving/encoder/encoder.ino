#define IN1 6
#define IN2 7
#define IN3 9
#define IN4 10
#define IR_sensor 2
#define ENCODER_A 3
#define ENCODER_B 4
const float WHEEL_CIRCUMFERENCE = 20.4; // circumference = pi * D
const int PULSES_PER_REVOLUTION = 400 ;
const float DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REVOLUTION;
const int MOTOR_RPM = 130;
volatile int encoderCount = 0;

void setup() {
 pinMode(IN1 , OUTPUT);
 pinMode(IN2 , OUTPUT);
 pinMode(IN3 , OUTPUT);
 pinMode(IN4 , OUTPUT);
 pinMode(IR_sensor , INPUT);
 pinMode(ENCODER_A , INPUT);
 pinMode(ENCODER_B , INPUT);
 attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulses, RISING);


}

void loop() {
 moveForwardDistance(40);
 if(digitalRead(IR_sensor) == 0) {
   stopMotors();
   avoidObstacle();

 }
 turnAngle(90);

}
void countPulses() {
  encoderCount++;
}
void moveForwardDistance( int cm) {
 int targetPulses = cm / DISTANCE_PER_PULSE;
 encoderCount = 0;

  while (encoderCount < targetPulses) {
    if (digitalRead(IR_sensor) == 0) {
      
      stopMotors();
      avoidObstacle();
      break;
    }
     moveForward();
  }
  stopMotors();
}
void turnAngle(int angle) {
  int pulsesForFullTurn = (MOTOR_RPM * PULSES_PER_REVOLUTION) / 60; // Pulses for a full 360° turn
  int targetPulses = (angle / 360.0) * pulsesForFullTurn;
  encoderCount = 0;

  while (encoderCount < targetPulses) {
    turnRight();
  }
  stopMotors();
}
void avoidObstacle() {
  
  moveBackwardDistance(10);
  turnAngle(90);

}
void moveBackwardDistance(int cm) {
  int targetPulses = cm / DISTANCE_PER_PULSE;
  encoderCount = 0;

  while (encoderCount < targetPulses) {
    moveBackward();
  }
  stopMotors();
}
void moveForward(){
  digitalWrite(IN1 , HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , HIGH);
  digitalWrite(IN4 , LOW);
}
 void moveBackward(){
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , HIGH);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
 }
 void turnRight(){
  digitalWrite(IN1 ,HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
 }
void stopMotors(){
  digitalWrite(IN1 ,LOW);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , LOW);
}


