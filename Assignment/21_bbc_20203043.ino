#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// 서보의 각도
#define CENTER 1475
#define MAX 1350
#define MIN 1770
#define MID 1650

Servo myservo;

// 변수
float setMax, setMin, setMid, emaAlpha, emaValue, setCenter;
int count = 0;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  setCenter = CENTER;

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(setCenter);

  setMax = MAX;
  setMin = MIN;
  setMid = MID;
  
// initialize serial port
  Serial.begin(57600);
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR))/100.0;
  if (volt > 4.20) {
    val = (0.0407*volt*volt)-(2.9232*volt)+26.559332;
  }
  else if (volt > 3.05) {
    val = (9.3776*volt*volt)-(76.6832*volt)+171.648576;
  }
  else if (volt > 2.68) {
    val = -(25*volt)+101.25;
  }
  else {
    val = (166.66666*volt*volt)-(891.66663*volt)+1227.49995;
  }
  return val;
}


void loop() {
  float raw_dist = ir_distance();
  while(count < 3000){
    Serial.print("count : ");
    Serial.print(count);
    Serial.print(", servo : ");
    Serial.println(myservo.read());
    count++;
  }
 
//  while(1){
  //  Serial.println(myservo.read());
    //myservo.writeMicroseconds(setMid);}

  if(raw_dist < 25.50){
    myservo.writeMicroseconds(setMin);
    digitalWrite(PIN_LED, 0);
  } else if(raw_dist > 25.50){
    myservo.writeMicroseconds(setMax);
    digitalWrite(PIN_LED, 1);
  }    


  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",val:");
  Serial.print(floor(raw_dist + 0.5));
  Serial.print(",volt:");
  Serial.print(float(analogRead(PIN_IR)));
  Serial.print(",servo:");
  Serial.println(myservo.read());
  
  
  delay(20);
}
