
#include <ros.h>
#include <std_msgs/Float32.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


ros::NodeHandle nh;

#define enc_count_rev 12
#define ENC_IN 18

//int PWM=4;
//int DIR=2;

int in1=2; //
int in2=4;
int enA=3; //pwm

int index=0;
float total=0.0;
float average=0.0;
const int numReadings=300;
float readings[numReadings];

float rpm=0;
float rpmFilter=rpm;
float alpha=0.05;
float rad=1;
float motorPWM=0;
long time= 0.0;
long previousTime= 0;
bool encoderDirection=false;
bool direction=false;


void messageCb(const std_msgs::Float32& val_msg){
  //motorPWM=map(val_msg.data,-1,1,-255,255);
  motorPWM=val_msg.data*255;
  if (motorPWM > 255){
    analogWrite(enA, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (motorPWM<0){
    if (abs(motorPWM)>=255){
      //analogWrite(DIR, 255);
      analogWrite(enA, 255);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else{
      //analogWrite(DIR, abs(motorPWM));
      analogWrite(enA, abs(motorPWM));
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    
  }
  else{
    //analogWrite(PWM, motorPWM);
    analogWrite(enA, motorPWM);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
}

void motor_pulse(){
  long currentTime=micros();
  long time= currentTime-previousTime;
  int val=digitalRead(encoderDirection);
  if(val==LOW){
    direction=true;
  }
  else{
    direction=false;
  }
  if(direction){
    rpm= -1*60000000.0/(time*enc_count_rev*35*285);
    //rpmFilter=alpha*rpm+(1.0-alpha)*rpmFilter;
  }
  else{
    rpm=60000000.0 /(time*enc_count_rev*35*285);
    //rpmFilter=alpha*rpm+(1.0-alpha)*rpmFilter;
  }
  previousTime = currentTime;  
}


std_msgs::Float32 rpm_msg;
ros::Publisher pub_motor_output("/rpm",&rpm_msg);
ros::Subscriber<std_msgs::Float32> sub("/u_out", &messageCb);


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);

  pinMode(ENC_IN, INPUT_PULLUP);

  //pinMode(PWM,OUTPUT);
  //pinMode(DIR,OUTPUT);
  for(int thisReading=0;thisReading<numReadings; thisReading++){
    readings[thisReading]=0;
  }
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_IN), motor_pulse, RISING);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_motor_output);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  total=total-readings[index];
  readings[index]=rpm;
  total=total+readings[index];
  index=index+1;  
  if (index>=numReadings){
    index=0;
  }
  average=total/numReadings;


  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  // Display static text
  display.println("AV-RPM");
  display.setTextSize(3);
  display.setCursor(20, 20);
  // Display static text
  display.println(average);
  display.display(); 

  
  rpm_msg.data=average;
  pub_motor_output.publish(&rpm_msg);
  nh.spinOnce();

}