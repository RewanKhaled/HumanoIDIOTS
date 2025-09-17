#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 8
#define IN3 9
#define IN4 10
#define IR_sensor 2
void setup() {
  pinMode(ENA , OUTPUT);
  pinMode(IN1 , OUTPUT);
  pinMode(IN2 , OUTPUT);
  pinMode(ENB , OUTPUT);
  pinMode(IN3 , OUTPUT);
  pinMode(IN4 , OUTPUT);
  pinMode(IR_sensor , INPUT);
  analogWrite(ENA , 150);
  analogWrite(ENB , 150);

}

void loop() {
  bool obstacledetection = digitalRead(IR_sensor);
  if (obstacledetection == 0 ){
    stop();
    delay(500);
    moveBackward();
    delay(500);
    turnRight();
    delay(500);
 
  }
  else {
    moveForward();
  }

 

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
  digitalWrite(IN1 , HIGH);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN4 , HIGH);
}
void stop(){
  digitalWrite(IN1 , LOW);
  digitalWrite(IN2 , LOW);
  digitalWrite(IN3 , LOW);
  digitalWrite(IN1 , LOW);
}
