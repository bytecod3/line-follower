// sensor connection
#define RIGHT_IR_IN 2
#define LEFT_IR_IN 3

// L293 motor driver conection
#define in1 4
#define in2 5

#define in3 6
#define in4 7

#define ena 10
#define enb 11

// motor speeds
const int speed = 45;
const int left_motor_speed = 30; // left motor runs at a slightly higher speed compared to the right motor
const int turn_speed = 20;
const int my_delay = 50;

//=======================PID Control variables==============================
float kp;  // proportional term
float kd;  // derivative term
float old_error = 0;  
float new_error; 
float old_time = millis();
float new_time;
float dt; // change in error
float d_e; // change in time
float correction; // correction value
float reference_position;  // the reference point -> ideal point where we want to be always

float pid(float kp, float kd, float old_error, float new_error, float old_time, float new_time, float pos_l2, float pos_r2, int l2_pin){
/*
* calculate the correction to be applied to the motor movement
*/

  new_time = millis(); 
  dt = new_time - old_time;  // get the elapsed time
  old_time = new_time;  // save time to use in the next loop

  // get error and change in error 
  pos_l2 = analogRead(L2_IR_IN); // get the current position from the left middle sensor
  pos_r2 = analogRead(R2_IR_IN);

  error = reference_position - pos_l2;  
  d_e = new_error - old_error;  
  old_error = new_error;
  
  correction = (kp * error) * (kd * ());

  return correction;

}

void forward(){

  // turn ON all motors  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // left motor on 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(ena, speed);
  analogWrite(enb, speed);

}

void left(){
  // turn motors left - right motor on, left motor reversed direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  

  analogWrite(ena, turn_speed);
  analogWrite(enb, speed);

}

void right(){
  // turn motors right
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  

  analogWrite(ena, speed);
  analogWrite(enb, turn_speed);

}

void stop(){
  stop()

  // turn motors OFF
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RIGHT_IR_IN, INPUT);
  pinMode(LEFT_IR_IN, INPUT);

  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  // all motors initially off

  delay(2000);

}

void loop() {
  int right_val = digitalRead(RIGHT_IR_IN);
  int left_val = digitalRead(LEFT_IR_IN);

  //Serial.print("Left: ");Serial.print(right_val);Serial.print("\t\tRight: ");Serial.print(left_val); Serial.println();
  
  if((right_val == 1) && (left_val == 0)){
    // steer to the left
    left();
  }
  
  else if((right_val == 0) && (left_val == 1)){
    // steer to the right
    right();
  }
 
  else if((right_val == 0) && (left_val == 0)){
    forward();
  } 

  else if((right_val == 1) && (left_val == 1)){
    stop();
  } 

  delay(my_delay);

}
