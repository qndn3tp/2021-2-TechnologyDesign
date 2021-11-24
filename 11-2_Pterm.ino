// 11-2 P제어(템플릿코드)

#include <Servo.h> 

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10 
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 255 // [2952] 탁구공의 목표 위치를 25.5cm로 설정
#define _DIST_MIN 100 // [2972] 거리 센서가 인식 가능하게 설정한 최소 거리
#define _DIST_MAX 410 // [2972] 거리 센서가 인식 가능하게 설정한 최대 거리

// Distance sensor
#define _DIST_ALPHA 0.2

// Servo range
#define _DUTY_MIN 1000 // [2952] 서보의 최소 각도값
#define _DUTY_NEU 1500 // [2952] 서보의 중간 각도값
#define _DUTY_MAX 2000 // [1691] 서보의 최대 각도

// Servo speed control
#define _SERVO_ANGLE 30  //[2967] 서보 각도 설정
#define _SERVO_SPEED 30  //[2959] 서보의 속도 설정

// Event periods
#define _INTERVAL_DIST 20  //[2959] 센서의 거리측정 인터벌값
#define _INTERVAL_SERVO 20 //[2967] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  //[2959] 시리얼 모니터/플로터의 인터벌값 설정

// PID parameters
#define _KP 0.7// [2961] 비례이득

// 실제 거리가 100mm, 400mm일 때 센서가 읽는 값(각 a, b)
#define a 68.19
#define b 291.47

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    // [2961] dist_raw : 적외선센서로 얻은 거리를 저장하는 변수
                             // [2961] dist_ema : 거리를 ema필터링을 한 값을 저장하는 변수
float alpha;    // [2959] ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // [2957] last_sampling_time_dist : 거리센서 측정 주기
                                                                                            // [2957] last_sampling_time_servo : 서보 위치 갱신 주기
                                                                                            // [2957] last_sampling_time_serial : 제어 상태 시리얼 출력 주기
bool event_dist, event_servo, event_serial; // [2957] 각각의 주기에 도달했는지를 불리언 값으로 저장하는 변수

// Servo speed control
int duty_chg_per_interval; // [2952] 한 주기당 변화할 서보 활동량을 정의
int duty_target, duty_curr; // [2961] 목표위치, 서보에 입력할 위치

// PID variables
float error_curr, error_prev, up_control, down_control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED, OUTPUT);
myservo.attach(PIN_SERVO);

// initialize global variables
alpha = _DIST_ALPHA;   // [2959] ema의 알파값 초기화
dist_ema = 0;          // [2959] dist_ema 초기화
dist_target = _DIST_TARGET; // dist_target 초기화
// duty_neutral = _DUTY_NEU;

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);// [2952] 서보 모터를 중간 위치에 지정
delay(1000);

// initialize serial port
Serial.begin(115200);

// convert angle speed into duty change per interval.
duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);  // [2959] 한 주기마다 이동할 양
                                                                                                        // [2974] INTERVAL -> _INTERVAL_SERVO 로 수정

// [2974] 이벤트 변수 초기화
last_sampling_time_dist = 0; // [2974] 마지막 거리 측정 시간 초기화
last_sampling_time_servo = 0; // [2974] 마지막 서보 업데이트 시간 초기화
last_sampling_time_serial = 0; // [2974] 마지막 출력 시간 초기화
event_dist = event_servo = event_serial = false; // [2974] 각 이벤트 변수 false로 초기화
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();  // [2964] event 발생 조건 설정
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true; // [2957] 거리 측정 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true; // [2957] 서보모터 제어 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true; // [2957] 출력주기에 도달했다는 이벤트 발생
  }

////////////////////
// Event handlers //
////////////////////

  // 거리측정주기에 도달
  if(event_dist) { 
     event_dist = false;
      dist_raw = ir_distance_filtered();   // [2959] dist_raw에 필터링된 측정값 저장
      if (dist_ema == 0){            // [2959] 맨 처음
        dist_ema = dist_raw;               // [2959] 맨 처음 ema값 = 필터링된 측정값
      }                                    // [2963] dist_ema를 dist_raw로 초기화
      else{
        dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;   // [2959] ema 구현
      }  
   
  // PID control logic
   error_curr = dist_target - dist_ema;
   pterm = error_curr;
   down_control = _KP * pterm;     // 기울기가 +, -인지에 따라 _KP에 펄스폭 차이반영
   up_control = (1.1 * _KP) *pterm;

  // duty_target = f(duty_neutral, control)
    if (pterm > 0) {
      duty_target =  _DUTY_NEU + down_control;
    }
    else {
      duty_target = _DUTY_NEU + up_control;
    }

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX;
    }
    else if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;
    }
  
 }

  // 서보측정주기에 도달
  if(event_servo) {
    event_servo = false; // [2974] 서보 이벤트 실행 후, 다음 주기를 기다리기 위해 이벤트 종료
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {  // [2964] 현재 서보 각도 읽기
      duty_curr += duty_chg_per_interval; // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 증가
      if(duty_curr > duty_target) {duty_curr = duty_target;} // [2956] duty_target 지나쳤을 경우, duty_target에 도달한 것으로 duty_curr값 재설정
    }
    else {
      duty_curr -= duty_chg_per_interval;  // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 감소
      if (duty_curr < duty_target) {duty_curr = duty_target;} // [2956] duty_target 지나쳤을 경우, duty_target에 도달한 것으로 duty_curr값 재설정
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);  // [2964] 서보 움직임 조절

  }

  // 출력주기에 도달
  if(event_serial) {
    event_serial = false; // [2974] 출력 이벤트 실행 후, 다음 주기까지 이벤트 종료
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw); // [2957] 적외선 센서로부터 받은 값 출력
    Serial.print(",pterm:");
    Serial.print(pterm); // [2961] 비례제어량 출력
    Serial.print(",duty_target:"); 
    Serial.print(duty_target); // [2957] 목표로 하는 거리 출력
    Serial.print(",duty_curr:"); 
    Serial.print(duty_curr); // [2957] 서보모터에 입력한 값 출력
    Serial.println(",High:310,Max:2000");
  }
}
float ir_distance(void){ // return value unit: mm
                         // [2959] 센서가 측정한 거리를 리턴해주는 함수
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
  return val;             
}

float ir_distance_filtered(void){ // return value unit: mm
                           // [2959] 센서가 측정한 거리를 필터링한 값을 리턴
  float val = ir_distance();     // [2952] ir_distance()에서 리턴된 값을 val에 대입후 필터링 수행
  return 100 + 300.0 / (b - a) * (val - a);
}
