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

ros::NodeHandle  nh;
int x;

void cb( const std_msgs::Int32& cmd_msg){
  x = cmd_msg.data;
}

ros::Subscriber<std_msgs::Int32> rActuator("r_actuator", &cb);

std_msgs::Int32 x_msg;
ros::Publisher rSensor("r_sensor", &x_msg);

void setup(){
  nh.initNode();
  nh.subscribe(rActuator);
  nh.advertise(rSensor);
}

void loop(){
  x_msg.data = x;
  rSensor.publish(&x_msg);
  nh.spinOnce();
  delay(500);
}
