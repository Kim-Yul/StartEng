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

// 플레이트 측정
#define DIST_TARGET 275
#define DIST_MIN 100
#define DIST_MAX 400

// 서보 속도
#define SERVO_ANGLE 30
#define SERVO_SPEED 1000

// Event periods
#define EVENT_DIST 20 //적외선 센서
#define EVENT_SERVO 20 //서보
#define EVENT_SERIAL 100 //시리얼 플로터

//filter
#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

//ema
#define _DIST_ALPHA 0.42 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

// PID
#define KP 100.0
#define KD 1.0

Servo myservo;

// 변수
float dist_target, dist_raw;
float setMax, setMin, setMid, setCenter;
int count = 0;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// 서보 속도 조절
int duty_chg_per_interval;
int duty_target, duty_curr;

// filter
int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum, dist_ema, alpha;

// PID 변수
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  setCenter = CENTER;
  dist_target = DIST_TARGET;

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(setCenter);

  setMax = MAX;
  setMin = MIN;
  setMid = MID;
  
// initialize serial port
  Serial.begin(57600);

  a = 68;
  b = 245;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;

  duty_chg_per_interval = (float)(setMax - setMin)*(SERVO_SPEED / SERVO_ANGLE)*(EVENT_SERVO / 1000.0);
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR))/100.0;
  if (volt > 4.20) {
    val = (0.0407*volt*volt)-(2.9232*volt)+26.559332;
  } else if (volt > 3.05) {
    val = (9.3776*volt*volt)-(76.6832*volt)+171.648576;
  } else if (volt > 2.68) {
    val = -(25*volt)+101.25;
  } else {
    val = (166.66666*volt*volt)-(891.66663*volt)+1227.49995;
  }
  return val;
}


float ir_distence_filter(void) {
  sum = 0;
  iter = 0;
  while (iter < LENGTH) {
    dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }
   for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }

  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }
  float dist_cali = sum/(LENGTH-2*k_LENGTH);

  return alpha*dist_cali + (1-alpha)*dist_ema;
}
  

void loop() {
  float raw_dist = ir_distance();
  while(count < 1000){
    Serial.print("count : ");
    Serial.print(count);
    Serial.print(", servo : ");
    Serial.println(myservo.read());
    count++;
  }

  if(raw_dist < dist_target){
    myservo.writeMicroseconds(setMin);
    delay(10);
    digitalWrite(PIN_LED, 0);
  } else if(raw_dist > dist_target){
    myservo.writeMicroseconds(setMax);
    delay(10);
    digitalWrite(PIN_LED, 1);
  }

  event_dist = true;
//  Serial.println("dist");
  event_servo = true;
//  Serial.println("servo");
  event_serial = true;
//  Serial.println("serial");

 /*
  while(1){
    Serial.println(myservo.read());
    myservo.writeMicroseconds(setCenter);}
*/

  if(event_dist){
    event_dist = false;
    dist_ema = ir_distence_filter();



    // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = KP*error_curr;
    //dterm = KD*(error_curr - error_prev);
    control = pterm;

    duty_target = setMid + control;

  // 파손 방지를 위한 target 확인
    if(duty_target > setMax){
      duty_target = setMax;
    } else if(duty_target < setMin){
      duty_target = setMin;
    }

    error_prev = error_curr;
    /*
    Serial.print("ema: ");
    Serial.print(dist_ema);
    Serial.print(", pterm: ");
    Serial.print(pterm);
    Serial.print(", dterm: ");
    Serial.print(dterm);
    Serial.print(", control: ");
    Serial.print(control);
    Serial.print(", target: ");
    Serial.print(duty_target);
    Serial.print(", error_curr: ");
    Serial.print(error_curr);
    Serial.print(", error_prev: ");
    Serial.println(error_prev);*/
  }


  if(event_servo){
    event_servo = false;
    duty_curr = 0;

    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }/*
    Serial.print("interval: ");
    Serial.print(duty_chg_per_interval);
    Serial.print(", duty_curr: ");
    Serial.println(duty_curr);*/
    myservo.writeMicroseconds(duty_curr);
  }



  if(event_serial){
    event_serial = false;
    Serial.print("dist_ir : ");
    Serial.print(dist_raw);
    Serial.print(", pterm : ");
    Serial.print(map(pterm, -1000,1000,510,610));
    Serial.print(", duty_target : ");
    Serial.print(map(duty_target, 1000,2000,410,510));
    Serial.print(", duty_curr : ");
    Serial.print(map(duty_curr, 1000,2000,410,510));
    Serial.println(", Min : 100, Low : 200, dist_target : 275, Hight 310, Max : 410");
  }



  
  

/*
  if(raw_dist < 25.50){
    myservo.writeMicroseconds(setMin);
    digitalWrite(PIN_LED, 0);
  } else if(raw_dist > 25.50){
    myservo.writeMicroseconds(setMax);
    digitalWrite(PIN_LED, 1);
  }    
*/

  /*
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",val:");
  Serial.print(floor(raw_dist + 0.5));
  Serial.print(",volt:");
  Serial.print(float(analogRead(PIN_IR)));
  Serial.print(",servo:");
  Serial.println(myservo.read());
  */
  
  /*
  delay(20);
  */
}
