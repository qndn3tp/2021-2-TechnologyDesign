// 11-2 BangBang Control - ema필터 적용

#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// global variables
Servo myservo;
int a, b; // unit: mm
float alpha, dist_ema;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1500);
  delay(2000);
  
// initialize serial port
  Serial.begin(57600);
  a = 68; //70;
  b = 291; //300;
  alpha = 0.2;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  
  // ema 필터링
  dist_ema = alpha * dist_cali + (1-alpha)  * dist_ema;
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);

 // Bang-Bang Control
  if (dist_ema <= 255.0){
    myservo.writeMicroseconds(1680);
   }
  else {
    myservo.writeMicroseconds(1360);
  }
}
