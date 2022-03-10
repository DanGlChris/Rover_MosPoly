#include "motorLib.h"
#include "PPMReader.h"
#include <Arduino_FreeRTOS.h>
//#include <ros.h>
//#include "geometry_msgs/Twist.h"

#define MAX_CHANNEL 10
#define PPM_PIN 21

PPMReader ppm(PPM_PIN, 6);

myMotor motor1;//  1       4
myMotor motor2;//
myMotor motor3;//  2   ^   5
myMotor motor4;//
myMotor motor5;//  3       6
myMotor motor6;//

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
  
  DDRE |= (1 << 3) | (1 << 4) | (1 << 5);
  DDRH |= (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);
  DDRB |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
  DDRG |= (1 << 5);
   
  xTaskCreate(TaskMotorUpdate, "motorUpdate",
    128, NULL, 2, NULL);
  
  xTaskCreate(TaskNodeSpinOnce, "nodeSpinOnce",
    128, NULL, 1, NULL);
    
}

void loop() {
  move_forward_backward(50, true);
}

void TaskMotorUpdate(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    uint16_t throttle = ppm.latestValidChannelValue(3, 0);
    uint16_t turn = ppm.latestValidChannelValue(1, 0);
    Serial.println(throttle);
    vTaskDelay(500);
  }
}

void TaskNodeSpinOnce(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    //nh.spinOnce();
    Serial.println("test");
    vTaskDelay(50);
  }
}
void move_forward_backward(uint8_t pwm, bool route){
  motor1.setVelocity(pwm, route);
  motor2.setVelocity(pwm, route);
  motor3.setVelocity(pwm, route);
  motor4.setVelocity(pwm, route);
  motor5.setVelocity(pwm, route);
  motor6.setVelocity(pwm, route);
  delay(10);
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
