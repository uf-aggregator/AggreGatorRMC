#include <Wire.h>

unsigned long last_motor_write;

const int m_enable = ;
const int left_pwm = 10;
const int left_dira = 12;
const int left_dirb = 11;
const int right_pwm = 9;
const int right_dira = 7;
const int right_dirb = 8;

volatile char command;


void both_forward(){
  digitalWrite(left_dira, HIGH);
  digitalWrite(left_dirb, LOW);
  digitalWrite(right_dira, HIGH);
  digitalWrite(right_dirb, LOW);
}
void left_forward_right_backward(){
  digitalWrite(left_dira, HIGH);
  digitalWrite(left_dirb, LOW);
  digitalWrite(right_dira, LOW);
  digitalWrite(right_dirb, HIGH);
}
void left_backward_right_forward(){
  digitalWrite(left_dira, LOW);
  digitalWrite(left_dirb, HIGH);
  digitalWrite(right_dira, HIGH);
  digitalWrite(right_dirb, LOW);
}
void both_backward(){
  digitalWrite(left_dira, LOW);
  digitalWrite(left_dirb, HIGH);
  digitalWrite(right_dira, LOW);
  digitalWrite(right_dirb, HIGH);
  
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(1); //slave with address of 1
  Wire.onReceive(interpretCommand);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long curr_time = millis();
  if(curr_time - last_motor_write > 1000){
    analogWrite(left_pwm, 0);
    analogWrite(right_pwm, 0);
  }
}

void interpretCommand(int howMany){
  last_motor_write = millis();
  char temp;
  command = Wire.read(); //get first byte - command
  if(command == 0){
    //go forward
    both_forward();
    temp = Wire.read();
    analogWrite(left_pwm, temp);
    temp = Wire.read();
    analogWrite(right_pwm, temp);
  }
  if(command == 1){
    //left forward, right backward
    left_forward_right_backward();
    temp = Wire.read();
    analogWrite(left_pwm, temp);
    temp = Wire.read();
    analogWrite(right_pwm, temp);
  }
  if(command == 2){
    //left backward, right forward
    left_backward_right_forward();
    temp = Wire.read();
    analogWrite(left_pwm, temp);
    temp = Wire.read();
    analogWrite(right_pwm, temp);
  }
  if(command == 3){
    //left backward, right backward
    both_backward();
    temp = Wire.read();
    analogWrite(left_pwm, temp);
    temp = Wire.read();
    analogWrite(right_pwm, temp);
  }
  if(command == 4){
    //do something else to prepare for read
    
  }
}
