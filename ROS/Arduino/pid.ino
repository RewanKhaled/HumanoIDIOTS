#include <util/atomic.h>

// Pins
#define ENCA1 2
#define ENCB1 3
#define PWM1 6
#define IN1A 30
#define IN1B 32

// Pins for Motor 2
#define ENCA2 18
#define ENCB2 19
#define PWM2 5
#define IN2A 34
#define IN2B 36

// globals
long prevT1 = 0;
int posPrev1 = 0;
volatile int pos1_i = 0;
volatile float velocity1_i = 0;
volatile long prevT1_i = 0;
float v1Filt = 0;
float v1Prev = 0;
float eintegral1 = 0;

// Globals for Motor 2
long prevT2 = 0;
int posPrev2 = 0;
volatile int pos2_i = 0;
volatile float velocity2_i = 0;
volatile long prevT2_i = 0;
float v2Filt = 0;
float v2Prev = 0;
float eintegral2 = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1, RISING);

  // Motor 2 setup
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN2B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
}

void loop() {

  // Motor 1 control
  int pos1 = 0;
  float velocity1 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = pos1_i;
    velocity1 = velocity1_i;
  }
  long currT1 = micros();
  float deltaT1 = ((float)(currT1 - prevT1)) / 1.0e6;
  float velocity1_calc = (pos1 - posPrev1) / deltaT1;
  posPrev1 = pos1;
  prevT1 = currT1;
  float v1 = velocity1_calc / 744.0 * 60.0;
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  float vt1 = 100 * (sin(currT1 / 1e6) > 0);
  float e1 = vt1 - v1Filt;
  eintegral1 += e1 * deltaT1;
  float kp1 = 5, ki1 = 10;
  float u1 = kp1 * e1 + ki1 * eintegral1;
  int dir1 = (u1 < 0) ? -1 : 1;
  int pwr1 = (int)fabs(u1);
  if (pwr1 > 255) pwr1 = 255;
  setMotor1(dir1, pwr1, PWM1, IN1A, IN1B);

  // Motor 2 control
  int pos2 = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos2 = pos2_i;
    velocity2 = velocity2_i;
  }
  long currT2 = micros();
  float deltaT2 = ((float)(currT2 - prevT2)) / 1.0e6;
  float velocity2_calc = (pos2 - posPrev2) / deltaT2;
  posPrev2 = pos2;
  prevT2 = currT2;
  float v2 = velocity2_calc / 744.0 * 60.0;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;
  float vt2 = 100 * (cos(currT2 / 1e6) > 0);
  float e2 = vt2 - v2Filt;
  eintegral2 += e2 * deltaT2;
  float kp2 = 5, ki2 = 10;
  float u2 = kp2 * e2 + ki2 * eintegral2;
  int dir2 = (u2 < 0) ? -1 : 1;
  int pwr2 = (int)fabs(u2);
  if (pwr2 > 255) pwr2 = 255;
  setMotor2(dir2, pwr2, PWM2, IN2A, IN2B);

   Serial.print("Motor 1 Target: "); Serial.print(vt1);
  Serial.print(" RPM, Actual: "); Serial.print(v1Filt);
  Serial.print(" | Motor 2 Target: "); Serial.print(vt2);
  Serial.print(" RPM, Actual: "); Serial.println(v2Filt);

  delay(10);
}

void setMotor1(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(PWM1,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  } 
}
  // set motor2
  void setMotor2(int dir, int pwmVal, int pwm, int in3, int in4){
  analogWrite(PWM2,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);    
  }
}

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos1_i = pos1_i + increment;
  long currT1 = micros();
  float deltaT1 = ((float) (currT1 - prevT1_i))/1.0e6;
  velocity1_i = increment/deltaT1;
  prevT1_i = currT1;
}
 
 void readEncoder2(){
  // Read encoder B when ENCA rises
  int c = digitalRead(ENCB2);
  int increment2 = 0;
  if(c>0){
    // If B is high, increment forward
    increment2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment2 = -1;
  }
  pos2_i = pos2_i + increment2;
 long currT2 = micros();
  float deltaT2 = ((float) (currT2 - prevT2_i))/1.0e6;
  velocity2_i = increment2/deltaT2;
  prevT2_i = currT2;
  
} 