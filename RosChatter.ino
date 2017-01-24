/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

class NewHardware : public ArduinoHardware {
public : NewHardware () : ArduinoHardware (&Serial1, 57600){ };
}; ros::NodeHandle_<NewHardware> nh;

std_msgs::String str_msg;
ros::Publisher debug("debug", &str_msg);
char msg[30];
int messagedelay = 0;


float ultrasonic(void) {
  digitalWrite(23, LOW);
  delayMicroseconds(2);
  digitalWrite(23, HIGH);
  delayMicroseconds(10);
  digitalWrite(23, LOW);

  long duration = pulseIn(22,HIGH,1740);   // 1740 uS => 30 cm
  long distance_cm = duration / 58;

  if (distance_cm > 0 && distance_cm < 15) {
    return 0.0;
  }
  else if (distance_cm > 0 && distance_cm < 30) {
    return 0.5//(float) (distance_cm-15)/15.0;
  } 
  else {
    return 1.0;
  }
}

void twistMsg(const geometry_msgs::Twist& twist_msg) {
  messagedelay = 0;


  int lspd = (int) (twist_msg.linear.x * 255.0);
  int rspd = lspd;
  int rot = (int)(twist_msg.angular.z * 255.0);
  lspd -= rot;
  rspd += rot;
  
  //sprintf(msg, "lspd = %d", lspd);
  //str_msg.data = msg;
  //debug.publish(&str_msg);

  float dist_factor = ultrasonic();
  
  lspd = (int) lspd * dist_factor;
  rspd = (int) rspd * dist_factor;
  
  
  
  if (rspd > 255)
    rspd = 255;
  if (rspd < -255)
    rspd = -255;
  if (lspd > 255)
    lspd = 255;
  if (lspd < -255)
    lspd = -255;
  
  //sprintf(msg, "factor = %d", dist_factor);
  //str_msg.data = msg;
  //debug.publish(&str_msg);
  
  if (lspd < 0) {
    analogWrite(7, 0);
    analogWrite(6, -lspd);
  } else {
    analogWrite(6, 0);
    analogWrite(7, lspd);
  }
  if (rspd < 0) {
    analogWrite(3, 0);
    analogWrite(2, -rspd);
  } else {
    analogWrite(2, 0);
    analogWrite(3, rspd);
  }
}

ros::Subscriber<geometry_msgs::Twist> twist("cmd_vel", &twistMsg);

void setup()
{
  // LCHB-100 Bridge & LED
  pinMode(7, OUTPUT);   // 1REV
  pinMode(24, OUTPUT);  // 1EN
  pinMode(6, OUTPUT);   // 1FWD
  pinMode(3, OUTPUT);   // 2REV
  pinMode(25, OUTPUT);  // 2EN
  pinMode(2, OUTPUT);   // 2FWD
  pinMode(13, OUTPUT);  // Yellow LED
  digitalWrite(13, LOW);

  //HC-SR04 ultrasonic sensor
  pinMode(22, INPUT);   // Echo
  pinMode(23, OUTPUT);  // Trigger
  
  nh.initNode();
  nh.subscribe(twist);
  nh.advertise(debug);
  digitalWrite(24, HIGH);
  digitalWrite(25, HIGH);
}

void fail() {
  digitalWrite(13, HIGH);
  analogWrite(6, 0);
  analogWrite(7, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}

void loop()
{
  nh.spinOnce();
  delay(1);
  if (messagedelay > 2000) {
    fail();
  } else {
   digitalWrite(13, LOW); //led off
   messagedelay++;
  }
}
