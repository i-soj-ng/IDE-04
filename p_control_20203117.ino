#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

//Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

//Distance senseor
#define _DIST_ALPHA 0.5

//Servo range
#define _DUTY_MIN 1500
#define _DUTY_NEU 1200
#define _DUTY_MAX 900

//Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 30.0

//Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

//PID Parameters
#define _KP 0.4

#define a 70
#define b 390

//////////////////////
// global variables //
//////////////////////

//Servo instance
Servo myservo;

//Distance sensor
float dist_target;
float dist_raw, dist_ema;

//Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

//Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

//PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//===================================================
#define DELAY_MICROS  1500
#define EMA_ALPHA 0.35
float ema_dist=0;
float filtered_dist;
float samples_num = 3;

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
// ================
float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  dist_raw = ir_distance();
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
//===================================================

void setup() {
  //initialize GPIO pins for LED and attach servo
  myservo.attach(PIN_SERVO); //attach servo
  pinMode(PIN_LED, OUTPUT); //initialize GPIO pins

  //move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;

  //initialize serial port
  Serial.begin(57600);

  //convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO/1000.0);
}  

void loop() {
  /////////////////////
  // Event generator //
  /////////////////////

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }

  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }

  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  ////////////////////
  // Event handlers //
  ////////////////////

  if(event_dist){
    event_dist = false;
    dist_ema = filtered_ir_distance(); //get a distance reading from the distance sensor
  }

  //PID control logic
  error_curr = dist_ema - _DIST_TARGET;
  pterm = error_curr;
  iterm = 0;
  dterm = 0;
  control = _KP * (-pterm) + iterm + dterm;

  duty_target = _DUTY_NEU + control * ((control > 0) ? (_DUTY_MIN - _DUTY_NEU) : (_DUTY_NEU - _DUTY_MAX));

  if(duty_target > _DUTY_MIN){
    duty_target = _DUTY_MIN;
  }
  else if(duty_target < _DUTY_MAX){
    duty_target = _DUTY_MAX;
  }

  if(event_servo){
    event_servo = false;
    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else{
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    //update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if(event_serial){
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}
