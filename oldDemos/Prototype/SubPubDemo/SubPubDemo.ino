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
#include <std_msgs/Empty.h>
#include <SimpleDHT.h>

int C;   // temperature C readings are integers
int H;   // humidity readings are integers

ros::NodeHandle  nh;

SimpleDHT11 dht11(9);

void led_cb( const std_msgs::Int32& cmd_msg){
  analogWrite(11, cmd_msg.data);
  analogWrite(12, cmd_msg.data);
  analogWrite(13, cmd_msg.data);//toggle led  
}

ros::Subscriber<std_msgs::Int32> led("led", &led_cb);

std_msgs::Int32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

std_msgs::Int32 hum_msg;
ros::Publisher pub_hum("humidity", &hum_msg);

std_msgs::Int32 light_msg;
ros::Publisher pub_light("light", &light_msg);

byte temperature = 0;
byte humidity = 0;

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(led);
  nh.advertise(pub_temp);
  nh.advertise(pub_hum);
  nh.advertise(pub_light);
}

void loop(){
  dht11.read(&temperature, &humidity, NULL);
  
  temp_msg.data = temperature;
  pub_temp.publish(&temp_msg);

  hum_msg.data = humidity;
  pub_hum.publish(&hum_msg);

  light_msg.data = analogRead(0);
  pub_light.publish(&light_msg);
  
  nh.spinOnce();
  delay(1);
}
