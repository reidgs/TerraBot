/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void water1_cb( const std_msgs::Bool& cmd_msg){
  analogWrite(9, cmd_msg.data ? 255 : 0);
}

void water2_cb( const std_msgs::Bool& cmd_msg){
  analogWrite(10, cmd_msg.data ? 255 : 0);
}

void air_cb( const std_msgs::Bool& cmd_msg){
  analogWrite(11, cmd_msg.data ? 255 : 0);
}

ros::Subscriber<std_msgs::Bool> w1("water1", &water1_cb);
ros::Subscriber<std_msgs::Bool> w2("water2", &water2_cb);
ros::Subscriber<std_msgs::Bool> air("air", &air_cb);


void setup(){
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  nh.initNode();
  nh.subscribe(w1);
  nh.subscribe(w2);
  nh.subscribe(air);
}

void loop(){

  
  nh.spinOnce();
  delay(1);
}
