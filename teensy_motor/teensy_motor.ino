
#include <Wire.h>

unsigned long last_motor_write = 0;

const int led = 13;
const int m_enable = 12;

//M1
const int left_pwm = 23; 
const int left_dira = 8;
const int left_dirb = 9;
const int left_curr = 21;
//M2
const int right_pwm = 22;
const int right_dira = 10;
const int right_dirb = 11;
const int right_curr = 20;

//encoders
const int encoder0PinA = 7;
const int encoder0PinB = 6;
//const int right_enca = ;
//const int right_encb = ;

volatile char command;

volatile int encoder0Pos = 0;

void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  } else {  // must be a high-to-low edge on channel A                                       
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  } else { // Look for a high-to-low on channel B
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }

} 

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
  //Serial.begin(9600);
  
  //setup i2c
  Wire.begin(1); //slave with address of 1
  Wire.onReceive(interpretCommand);
  Wire.onRequest(returnData);
  
  pinMode(led, OUTPUT);

  //setup motor pins
  pinMode(m_enable, OUTPUT);
  pinMode(left_dira, OUTPUT);
  pinMode(left_dirb, OUTPUT);
  pinMode(right_dira, OUTPUT);
  pinMode(right_dirb, OUTPUT);
  
  //setup encoder pins
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
 // pinMode(right_enca, INPUT);
  //pinMode(right_encb, INPUT);
  
  //setup encoder interrupts
  attachInterrupt(encoder0PinA, doEncoderA, CHANGE);
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE);
  //attachInterrupt(right_enca, encoderRightA, CHANGE);
  //attachInterrupt(right_encb, encoderRightB, CHANGE); 
  
  digitalWrite(led, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long curr_time = millis();
  if(curr_time - last_motor_write > 1000){
    analogWrite(left_pwm, 0);
    analogWrite(right_pwm, 0);
    digitalWrite(led, LOW);
    digitalWrite(m_enable, LOW);
  }else{
    digitalWrite(led, HIGH);
    digitalWrite(m_enable, HIGH);
  }
  
  //Serial.print("encoder0Pos: ");
  //Serial.println(encoder0Pos, DEC);
  
}

void interpretCommand(int howMany){
  last_motor_write = millis();
  char temp;
  command = Wire.read(); //get first byte - command
  digitalWrite(led, HIGH);
  digitalWrite(m_enable, HIGH);
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
}


void returnData(){
  uint8_t encoder_buffer[4];
  int curr_left = encoder0Pos;
  encoder0Pos = 0;
  encoder_buffer[0] = (curr_left & 0xFF000000) >> 24;
  encoder_buffer[1] = (curr_left & 0x00FF0000) >> 16;
  encoder_buffer[2] = (curr_left & 0x0000FF00) >> 8;
  encoder_buffer[3] = (curr_left & 0xFF);  
  Wire.write(encoder_buffer, 4);
}



