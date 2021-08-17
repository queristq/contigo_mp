#include "lobot_servo.h"

int m1_dir = 23;
int m1_pwm = 3;
int m2_dir = 22;
int m2_pwm = 2;

int incomingByte =5;
int left_v = 0;
int right_v = 0;
int des_lv = 0;
int des_rv = 0;
const int forward_speed = 70;
const int turn_speed_diff = 30;
const int acc = 1;

void setup() {
  // lobot servo motor to control the pan tilt of tablet
  Serial1.begin(115200);

  // pin mode setup
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_pwm, OUTPUT);

  // init of the pin outputs
  digitalWrite(m1_dir, LOW);
  digitalWrite(m1_pwm, LOW);
  digitalWrite(m2_dir, LOW);
  digitalWrite(m2_pwm, LOW);  

  Serial.begin(57600);           // set up Serial library at 9600 bps
  Serial.println("Hello Arduino Code is starting");  // prints hello with ending line break 
}

void RightWheelVel(int v)  // v needs to be between -255 to 255
{
  if (v>0)
  {
    digitalWrite(m2_dir, HIGH); 
    analogWrite(m2_pwm, v);
  }
  else
  {
    digitalWrite(m2_dir, LOW); 
    analogWrite(m2_pwm, -v);
  }
}

void LeftWheelVel(int v)  // v needs to be between -255 to 255
{
  if (v>0)
  {
    digitalWrite(m1_dir, HIGH); 
    analogWrite(m1_pwm, v);
  }
  else
  {
    digitalWrite(m1_dir, LOW); 
    analogWrite(m1_pwm, -v);
  }
}

void WheelVel(int right_v, int left_v)
{
  RightWheelVel(right_v);
  LeftWheelVel(left_v);
}

int pan_ang = 500;
int tilt_ang = 600;
int stepsize_tablet_mount_movement = 1;

void loop() {
  int i = 0;
  int d = 4;

  int cmd_tab_mount = incomingByte/10;
  int cmd_wheel = incomingByte%10;

    // put your main code here, to run repeatedly:

  switch (cmd_tab_mount)
  {
    case 7:
      pan_ang = pan_ang + stepsize_tablet_mount_movement;
      tilt_ang = tilt_ang + stepsize_tablet_mount_movement;
      break;
    case 8:
      tilt_ang = tilt_ang + stepsize_tablet_mount_movement;
      break;
    case 9:
      pan_ang = pan_ang - stepsize_tablet_mount_movement;
      tilt_ang = tilt_ang + stepsize_tablet_mount_movement;
      break;
    case 4:
      pan_ang = pan_ang + stepsize_tablet_mount_movement;
      break;
    case 5:
      break;
    case 6:
      pan_ang = pan_ang - stepsize_tablet_mount_movement;
      break;
    case 1:
      pan_ang = pan_ang + stepsize_tablet_mount_movement;
      tilt_ang = tilt_ang - stepsize_tablet_mount_movement;
      break;
    case 2:
      tilt_ang = tilt_ang - stepsize_tablet_mount_movement;
      break;
    case 3:
      pan_ang = pan_ang - stepsize_tablet_mount_movement;
      tilt_ang = tilt_ang - stepsize_tablet_mount_movement;
      break;
    case 0:
      pan_ang = 500;
      tilt_ang = 600;
      break;
  }
  LobotSerialServoMove(Serial1, ID1, pan_ang, 30);
  LobotSerialServoMove(Serial1, ID2, tilt_ang, 30);
  
  switch (cmd_wheel) 
  {
    case 7:    // your hand is on the sensor
      des_rv = forward_speed + turn_speed_diff;
      des_lv = forward_speed - turn_speed_diff;
      break;
    case 8:    // your hand is on the sensor
      des_rv = forward_speed;
      des_lv = forward_speed;
      break;
    case 9:    // your hand is on the sensor
      des_rv = forward_speed - turn_speed_diff;
      des_lv = forward_speed + turn_speed_diff;
      break;
    case 4:    // your hand is on the sensor
      des_rv = forward_speed;
      des_lv = -forward_speed;
      break;
    case 5:    // your hand is on the sensor
      des_rv = 0;
      des_lv = 0;
      break;
    case 6:    // your hand is on the sensor
      des_rv = -forward_speed;
      des_lv = forward_speed;
      break;
    case 1:    // your hand is on the sensor
      des_rv = -forward_speed - turn_speed_diff;
      des_lv = -forward_speed + turn_speed_diff;
      break;
    case 2:    // your hand is on the sensor
      des_rv = -forward_speed;
      des_lv = -forward_speed;
      break;
    case 3:    // your hand is on the sensor
      des_rv = -forward_speed + turn_speed_diff;
      des_lv = -forward_speed - turn_speed_diff;
      break;
    case 0:
      left_v = 0;
      right_v = 0;
  }

  if(left_v > des_lv) left_v = left_v - acc;
  if(left_v < des_lv) left_v = left_v + acc;
  if(right_v > des_rv) right_v = right_v - acc;
  if(right_v < des_rv) right_v = right_v + acc;
  
  WheelVel(left_v, right_v);
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }

  delay(d);
}
