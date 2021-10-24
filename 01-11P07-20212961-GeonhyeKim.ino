#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 550 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1480 // servo neutral position (90 degree)
#define _DUTY_MAX 2410 // servo full counterclockwise position (180 degree)
#define _DIST_ALPHA 0.3 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float dist_val; //거리에 비례하여 연속적으로 변화하기 위한 설정값
float dist_ema, alpha; // ema적용
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_LED,LOW); // led가 꺼진상태로 진행
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  alpha = _DIST_ALPHA;
// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {

// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
// 지수가중 이동평균 필터 ema이용
  dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;

// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println(",Max:400");

// adjust servo position according to the USS read value

// add your code here!
Serial.read(); // 현재 각도 읽기

if(dist_raw < 180.0) {
     myservo.writeMicroseconds(_DUTY_MIN);
     pinMode(PIN_LED,LOW);
  }
  else if(dist_raw < 360.0){
    dist_val = dist_raw - 180.0;
     myservo.writeMicroseconds(_DUTY_MIN+dist_val*10.34); // 거리가 증가함에따라 각도가 연속적으로 변화
     pinMode(PIN_LED,HIGH); // 측정범위내에서 LED점등
     
  }
  else {
    myservo.writeMicroseconds(_DUTY_MAX);
    pinMode(PIN_LED,LOW);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
