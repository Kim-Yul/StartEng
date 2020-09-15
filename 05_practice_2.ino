#define PIN_LED 7
unsigned int counting, count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    ;
  }
  Serial.println("Start");
  count = toggle = counting = 0;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  Serial.println("0s");
  digitalWrite(PIN_LED, 0);
  delay(1000);
  Serial.println("1s");
  while(count <= 9){
    count += 1;
    Serial.println(++counting);
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100); 
  }
  Serial.println("2s");
  while(1) {
    digitalWrite(PIN_LED, 1);
  }
}

int toggle_state(int toggle) {
  return (toggle == 1) ? 0 : 1;
}
