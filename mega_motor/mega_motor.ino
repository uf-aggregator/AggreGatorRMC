#include <Wire.h>

unsigned long last_bucket_write = 0;
unsigned long last_ladder_write = 0;

const int led = 13;

//BUCKET MOTORS
//bucket lift
const int bucket_vdd = 22;
const int bucket_lift_enable = 23;
const int bucket_lift_dira = 24;
const int bucket_lift_dirb = 25;
const int bucket_lift_pwm = 3; 
const int bucket_lift_cs_num = 2; //adc numb, not pin numb
//bucket dump
const int bucket_dump_enable = 26;
const int bucket_dump_dira = 27;
const int bucket_dump_dirb = 28;
const int bucket_dump_pwm = 2;
const int bucket_dump_cs_num = 3; //adc num

//LADDER MOTORS
//ladder lift
const int ladder_vdd= 12;
const int ladder_lift_enable = 11; 
const int ladder_lift_dira = 10;
const int ladder_lift_dirb = 9;
const int ladder_lift_pwm = 8;
const int ladder_lift_cs_num = 0; //adc num
//ladder conv
const int ladder_conv_enable = 7;
const int ladder_conv_dira = 6;
const int ladder_conv_dirb = 5;
const int ladder_conv_pwm = 4;
const int ladder_conv_cs_num = 1; //adc num

volatile char command;

volatile int ladder_lift_cs_val = 0;
volatile int ladder_conv_cs_val = 0;
volatile int bucket_lift_cs_val = 0;
volatile int bucket_dump_cs_val = 0;


void bucket_lift_pos_dump_pos(){
  digitalWrite(bucket_lift_dira, HIGH);
  digitalWrite(bucket_lift_dirb, LOW);
  digitalWrite(bucket_dump_dira, HIGH);
  digitalWrite(bucket_dump_dirb, LOW);
}

void bucket_lift_pos_dump_neg(){
  digitalWrite(bucket_lift_dira, HIGH);
  digitalWrite(bucket_lift_dirb, LOW);
  digitalWrite(bucket_dump_dira, LOW);
  digitalWrite(bucket_dump_dirb, HIGH);
}

void bucket_lift_neg_dump_pos(){
  digitalWrite(bucket_lift_dira, LOW);
  digitalWrite(bucket_lift_dirb, HIGH);
  digitalWrite(bucket_dump_dira, HIGH);
  digitalWrite(bucket_dump_dirb, LOW);
}

void bucket_lift_neg_dump_neg(){
  digitalWrite(bucket_lift_dira, LOW);
  digitalWrite(bucket_lift_dirb, HIGH);
  digitalWrite(bucket_dump_dira, LOW);
  digitalWrite(bucket_dump_dirb, HIGH);
}

void ladder_lift_pos_conv_pos(){
  digitalWrite(ladder_lift_dira, HIGH);
  digitalWrite(ladder_lift_dirb, LOW);
  digitalWrite(ladder_conv_dira, HIGH);
  digitalWrite(ladder_conv_dirb, LOW);
}

void ladder_lift_pos_conv_neg(){
  digitalWrite(ladder_lift_dira, HIGH);
  digitalWrite(ladder_lift_dirb, LOW);
  digitalWrite(ladder_conv_dira, LOW);
  digitalWrite(ladder_conv_dirb, HIGH);
}

void ladder_lift_neg_conv_pos(){
  digitalWrite(ladder_lift_dira, LOW);
  digitalWrite(ladder_lift_dirb, HIGH);
  digitalWrite(ladder_conv_dira, HIGH);
  digitalWrite(ladder_conv_dirb, LOW);
}

void ladder_lift_neg_conv_neg(){
  digitalWrite(ladder_lift_dira, LOW);
  digitalWrite(ladder_lift_dirb, HIGH);
  digitalWrite(ladder_conv_dira, LOW);
  digitalWrite(ladder_conv_dirb, HIGH);
}

void setup() {
  //setup i2c
  Wire.begin(2); //slave with address of 2
  Wire.onReceive(interpretCommand);
  Wire.onRequest(returnData);

  pinMode(led, OUTPUT);

  //setup motor pins
  pinMode(bucket_vdd, OUTPUT);
  digitalWrite(bucket_vdd, HIGH); //keep vdd pin high at 3.3V
  pinMode(bucket_lift_enable, OUTPUT);
  pinMode(bucket_dump_enable, OUTPUT);
  pinMode(bucket_lift_dira, OUTPUT);
  pinMode(bucket_lift_dirb, OUTPUT);
  pinMode(bucket_dump_dira, OUTPUT);
  pinMode(bucket_dump_dirb, OUTPUT);
  
  pinMode(ladder_vdd, OUTPUT);
  digitalWrite(ladder_vdd, HIGH); //keep vdd pin high at 3.3V
  pinMode(ladder_lift_enable, OUTPUT);
  pinMode(ladder_conv_enable, OUTPUT);
  pinMode(ladder_lift_dira, OUTPUT);
  pinMode(ladder_lift_dirb, OUTPUT);
  pinMode(ladder_conv_dira, OUTPUT);
  pinMode(ladder_conv_dirb, OUTPUT);

  digitalWrite(led, LOW);
}

void loop() {
  //main program only checks for timeouts
  
  unsigned long curr_time = millis();
  if(curr_time - last_bucket_write > 1000){
    //if no bucket message is received in past second, disable bucket system
    analogWrite(bucket_lift_pwm, 0);
    analogWrite(bucket_dump_pwm, 0);
    digitalWrite(bucket_lift_enable, LOW);
    digitalWrite(bucket_dump_enable, LOW);
    digitalWrite(led, LOW);
  }else{
    digitalWrite(bucket_lift_enable, HIGH);
    digitalWrite(bucket_dump_enable, HIGH);
  }
  
  curr_time = millis();
  if(curr_time - last_ladder_write > 1000){
    //if no ladder message is received in past second, disable ladder system
    analogWrite(ladder_lift_pwm, 0);
    analogWrite(ladder_conv_pwm, 0);
    digitalWrite(ladder_lift_enable, LOW);
    digitalWrite(ladder_conv_enable, LOW);
    digitalWrite(led, LOW);
  }else{
    digitalWrite(ladder_lift_enable, HIGH);
    digitalWrite(ladder_conv_enable, HIGH);
  }
  
  ladder_lift_cs_val = analogRead(ladder_lift_cs_num);
  ladder_conv_cs_val = analogRead(ladder_lift_cs_num);
  bucket_lift_cs_val = analogRead(bucket_lift_cs_num);
  bucket_dump_cs_val = analogRead(bucket_dump_cs_num);
  
}

void returnData(){
  //to send multiple bytes, 
  //you  must create a buffer and send it all at once
  uint8_t i2c_buffer[8];
  i2c_buffer[0] = ladder_lift_cs_val >> 8;
  i2c_buffer[1] = ladder_lift_cs_val & 0xFF;
  i2c_buffer[2] = ladder_conv_cs_val >> 8;
  i2c_buffer[3] = ladder_conv_cs_val & 0xFF;
  i2c_buffer[4] = bucket_lift_cs_val >> 8;
  i2c_buffer[5] = bucket_lift_cs_val & 0xFF;
  i2c_buffer[6] = bucket_dump_cs_val >> 8;
  i2c_buffer[7] = bucket_dump_cs_val & 0xFF;
  Wire.write(i2c_buffer, 8);
  
}

void interpretCommand(int howMany){
  char temp;
  command = Wire.read(); //get first byte - command
  
  if(command == 0 || command == 1 || command == 2 || command == 3){
    digitalWrite(bucket_lift_enable, HIGH);
    digitalWrite(bucket_dump_enable, HIGH);
    last_bucket_write = millis();
  }
  if(command == 4 || command == 5 || command == 6 || command == 7){
    digitalWrite(ladder_lift_enable, HIGH);
    digitalWrite(ladder_conv_enable, HIGH);
    last_ladder_write = millis();
  }

  digitalWrite(led, HIGH);
  
  if(command == 0){
    //bucket system - both positive
    bucket_lift_pos_dump_pos();
    temp = Wire.read();
    analogWrite(bucket_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(bucket_dump_pwm, temp);
  }
  if(command == 1){
    //bucket system - lift positive, dump negative
    bucket_lift_pos_dump_neg();
    temp = Wire.read();
    analogWrite(bucket_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(bucket_dump_pwm, temp);
  }
  if(command == 2){
    //bucket system - lift negative, dump positive
    bucket_lift_neg_dump_pos();
    temp = Wire.read();
    analogWrite(bucket_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(bucket_dump_pwm, temp);
  }
  if(command == 3){
    //bucket system - lift negative, dump negative
    bucket_lift_neg_dump_neg();
    temp = Wire.read();
    analogWrite(bucket_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(bucket_dump_pwm, temp);
  }
  if(command == 4){
    //ladder system - lift positive, convey positive
    ladder_lift_pos_conv_pos();
    temp = Wire.read();
    analogWrite(ladder_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(ladder_conv_pwm, temp);
  }
  if(command == 5){
    //ladder system - lift positive, convey negative
    ladder_lift_pos_conv_neg();
    temp = Wire.read();
    analogWrite(ladder_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(ladder_conv_pwm, temp);
  }    
  if(command == 6){
    //ladder system - lift negative, convey positive
    ladder_lift_neg_conv_pos();
    temp = Wire.read();
    analogWrite(ladder_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(ladder_conv_pwm, temp);
  }    
  if(command == 7){
    //ladder system - lift negative, convey negative
    ladder_lift_neg_conv_neg();
    temp = Wire.read();
    analogWrite(ladder_lift_pwm, temp);
    temp = Wire.read();
    analogWrite(ladder_conv_pwm, temp);
  }
  
}
