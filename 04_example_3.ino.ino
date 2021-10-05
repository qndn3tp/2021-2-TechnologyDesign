#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello world");
  count = toggle =0;
  digitalWrite(PIN_LED, toggle); //led는 꺼진상태로 초기화

}

void loop() {
  Serial.println(++count);
  toggle = count % 2; //toggle의 값은 count에 따라 0,1 반
  digitalWrite(PIN_LED, toggle);
  delay(1000);
}
