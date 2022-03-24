#include "motorLib.h"
#include "PPMReader.h"
#include <Arduino_FreeRTOS.h>

#include <Wire.h>
#include <Servo.h>

//#include <ros.h>
//#include "geometry_msgs/Twist.h"

/*#define MAX_CHANNEL 10
#define PPM_PIN 21*/

Servo servo_1, servo_2;  // create servo object to control a servo
int servo_pin1 = 18;
int servo_pin2 = 19;

//PPMReader ppm(PPM_PIN, 6);

myMotor motor1;//  1       4
myMotor motor2;//
myMotor motor3;//  2   ^   5
myMotor motor4;//
myMotor motor5;//  3       6
myMotor motor6;//

int ch1=14;  //channel 1 of receiptor
int ch2=15;  //channel 2 of receiptor
int ch3=16;  //channel 3 of receiptor
int ch4=17;  //channel 4 of receiptor


// value of frequency received
int Ch1_Value = 0, Ch1_V;  
int Ch2_Value = 0, Ch2_V;
int Ch3_Value = 0, Ch3_V;
int Ch4_Value = 0, Ch4_V;


//tampon value

int Ch1_Value_tampon = 0;
int Ch2_Value_tampon = 0;
int Ch3_Value_tampon = 0;
int Ch4_Value_tampon = 0;

int Value_Y_1 = 0, Value_Y_2 = 0, Value_Y_prctg_1 = 0, Value_Y_prctg_2 = 0;
int Value_R_1 = 0, Value_R_2 = 0, Value_R_prctg_1 = 0, Value_R_prctg_2 = 0;

//Joystick
boolean joystick_active = false;
boolean motor_forward = false;

void TaskMotorUpdate( void *pvParameters );
void TaskNodeSpinOnce( void *pvParameters ); 
//ros::NodeHandle nh;

/*void velCallback(  const geometry_msgs::Twist& vel)
{
     float x = (float)(vel.linear.x - 1.0); // I CAN USE VEL AS I WANT
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);*/


void setup() {
  initMotors();
  Serial.begin(9600);
  //nh.initNode();
  //nh.subscribe(sub);
  
 /* DDRE |= (1 << 3) | (1 << 4) | (1 << 5);
  DDRH |= (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
  DDRB |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
  DDRG |= (1 << 5);*/

  // pinMode receiver
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  
  servo_1.attach(servo_pin1);
  servo_2.attach(servo_pin2);
  servo_1.write(90);
  servo_2.write(90);
   
 /* xTaskCreate(TaskMotorUpdate, "motorUpdate",
    128, NULL, 2, NULL);
  
  xTaskCreate(TaskNodeSpinOnce, "nodeSpinOnce",
    128, NULL, 1, NULL);*/
    
}

void loop() {
  
  Ch1_V = pulseIn(ch1, HIGH);
  Ch2_V = pulseIn(ch2, HIGH);
  Ch3_V = pulseIn(ch3, HIGH);
  Ch4_V = pulseIn(ch4, HIGH);

  Ch1_V = transform_radio_frequency_pourcentage(Ch1_V);
  Ch1_Value = Ch1_V;
  
  Ch2_V = transform_radio_frequency_pourcentage(Ch2_V);
  Ch2_Value = Ch2_V;
  
  Ch3_V = transform_radio_frequency_pourcentage(Ch3_V);
  Ch3_Value = Ch3_V;  

  Ch4_V = transform_radio_frequency_pourcentage(Ch4_V);
  Ch4_Value = Ch4_V;

  if(!(Ch1_Value>=(Ch1_Value_tampon+5) || Ch1_Value<=(Ch1_Value_tampon-5))){
    Ch1_Value = Ch1_Value_tampon;
    joystick_active = false;
  }
  if(!(Ch2_Value>=(Ch2_Value_tampon+5) || Ch2_Value<=(Ch2_Value_tampon-5))){
    Ch2_Value = Ch2_Value_tampon;
    joystick_active = false;
  }
  if(!(Ch3_Value>=(Ch3_Value_tampon+5) || Ch3_Value<=(Ch3_Value_tampon-5))){
    Ch3_Value = Ch3_Value_tampon;
    joystick_active = false;
  }
  if(!(Ch4_Value>=(Ch4_Value_tampon+5) || Ch4_Value<=(Ch4_Value_tampon-5))){
    Ch4_Value = Ch4_Value_tampon;
    joystick_active = false;
  }
  
  Serial.print("channe4 : ");
  Serial.println(Ch4_Value);
  Y_move(Ch2_Value);
  wheel_rotation(servo_1, Ch4_Value);
  wheel_rotation(servo_2, Ch4_Value);
  
  delay(8);
  
  Ch1_Value_tampon = Ch1_Value;
  Ch2_Value_tampon = Ch2_Value;
  Ch3_Value_tampon = Ch3_Value;
  Ch4_Value_tampon = Ch4_Value;
}

/*void TaskMotorUpdate(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    uint16_t throttle = ppm.latestValidChannelValue(3, 0);
    uint16_t turn = ppm.latestValidChannelValue(1, 0);
    Serial.println(throttle);
    vTaskDelay(500);
  }
}*/

/*void TaskNodeSpinOnce(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    //nh.spinOnce();
    Serial.println("test");
    vTaskDelay(50);
  }
}*/
/**
 * the value of pwm is between 0 and 100
 */
void move_forward_backward(uint8_t pwm, bool route){
  motor1.setVelocity(pwm, route);
  motor2.setVelocity(pwm, route);
  motor3.setVelocity(pwm, route);
  motor4.setVelocity(pwm, route);
  motor5.setVelocity(pwm, route);
  motor6.setVelocity(pwm, route);
}
void stop_motors(){
  motor1.setStop();
  motor2.setStop();
  motor3.setStop();
  motor4.setStop();
  motor5.setStop();
  motor6.setStop();
}

int transform_radio_frequency_pourcentage(int Ch_Value){
  return map(Ch_Value, 985, 1995, -100, 100);
}

int transform_radio_pourcentage_frequency(int Ch_Value){
  return map(Ch_Value, -100, 100, 995, 1995);
}
/**
 * the value is on pourcentage
 */
void rotate_wheel(Servo servo_, int value_rotation){  
}

void Y_move(int velocity){
  //in this function motors A and B are the same
  //and motors C and D are the same
  if(velocity>100){ Value_Y_prctg_1=100; Value_Y_prctg_2=0; //to avoid inconvertable value
  }else if(velocity<-100){ Value_Y_prctg_1=0; Value_Y_prctg_2=100; //to avoid inconvertable value
  }
  
  else if(velocity>-5 && velocity<5){
    Value_Y_prctg_1=0;  //middle, value equal to zero
    Value_Y_prctg_2=0;
  }else if(velocity>=8){
     Value_Y_prctg_1 = velocity; // when the joystick exceeded last positive value he changes
     Value_Y_prctg_2 = 0; // the other pair of motor don't move
  }else if(velocity<=-8){
     Value_Y_prctg_1 = 0; // when the joystick exceeded last positive value he changes
     Value_Y_prctg_2 = abs(velocity); // the other pair of motor don't move
  }
  
  if(Value_Y_prctg_1!=0 && Value_Y_prctg_2 ==0 ){
    move_forward_backward(Value_Y_prctg_1, false);    
  }else if(Value_Y_prctg_1 == 0 && Value_Y_prctg_2!=0){
    move_forward_backward(Value_Y_prctg_2, true);
  }else if (Value_Y_prctg_1 == 0 && Value_Y_prctg_2 == 0){
    stop_motors();
  }
}
void wheel_rotation(Servo servo_, int rotation_angle){
  //in this function motors A and B are the same
  //and motors C and D are the same
  if(rotation_angle>100){ Value_R_prctg_1=100; Value_R_prctg_2=0; //to avoid inconvertable value
  }else if(rotation_angle<-100){ Value_R_prctg_1=0; Value_R_prctg_2=-100; //to avoid inconvertable value
  }
  
  else if(rotation_angle>-5 && rotation_angle<5){
    Value_R_prctg_1=0;  //middle, value equal to zero
    Value_R_prctg_2=0;
  }else if(rotation_angle>=5){
     Value_R_prctg_1 = rotation_angle; // when the joRstick exceeded last positive value he changes
     Value_R_prctg_2 = 0; // the other pair of motor don't move
  }else if(rotation_angle<=-5){
     Value_R_prctg_1 = 0; // when the joystick exceeded last positive value he changes
     Value_R_prctg_2 = rotation_angle; // the other pair of motor don't move
  }
  
  if(Value_R_prctg_1!=0 && Value_R_prctg_2 ==0 ){ 
    servo_.write(map(Value_R_prctg_1, -100, 100, 0, 180));
    Serial.println("Pass this");
  }else if(Value_R_prctg_1 == 0 && Value_R_prctg_2!=0){
    servo_.write(map(Value_R_prctg_2, -100, 100, 0, 180));
  }else if (Value_R_prctg_1 == 0 && Value_R_prctg_2 == 0){
    servo_.write(map(0, -100, 100, 0, 180));    
  }
}

void initMotors()
{
  motor1.init(2, 3);
  motor2.init(4, 5);
  motor3.init(6, 7);
  motor4.init(8, 9);
  motor5.init(10, 11);
  motor6.init(12, 13);
  
}
