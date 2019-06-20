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
#include <std_msgs/Bool.h>
#include <SimpleDHT.h>

//Is this necessary?
int C;   // temperature C readings are integers
int H;   // humidity readings are integers

ros::NodeHandle  nh;
int DHT_pin = 3;
int light_pin = 4;
int level_pin = 5;
int tds_pin = 6;
int led_pin = 7;
int wpump_pin = 8;
int npump_pin = 9;
int apump_pin = 10;
SimpleDHT11 dht11(DHT_pin);



// Functions for Actuators
void led_activate( const std_msgs::Int32& cmd_msg){
  analogWrite(led_pin, cmd_msg.data);//toggle led
}

void wpump_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(wpump_pin, cmd_msg.data ? 255 : 0);
}

void npump_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(npump_pin, cmd_msg.data ? 255 : 0);
}

void apump_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(apump_pin, cmd_msg.data ? 255 : 0);
}

// Actuators
ros::Subscriber<std_msgs::Int32> led_sub("led_raw", &led_activate);
ros::Subscriber<std_msgs::Bool> wpump_sub("wpump_raw", &wpump_activate);
ros::Subscriber<std_msgs::Bool> npump_sub("npump_raw", &npump_activate);
ros::Subscriber<std_msgs::Bool> apump_sub("apump_raw", &apump_activate);

// Sensors

std_msgs::Int32 humid_msg;
ros::Publisher humid_pub("humid_raw", &humid_msg);

std_msgs::Int32 temp_msg;
ros::Publisher temp_pub("temp_raw", &temp_msg);

std_msgs::Int32 light_msg;
ros::Publisher light_pub("light_raw", &light_msg);

std_msgs::Int32 level_msg;
ros::Publisher level_pub("level_raw", &level_msg);

std_msgs::Int32 tds_msg;
ros::Publisher tds_pub("tds_raw", &tds_msg);

void setup(){
  pinMode(led_pin, OUTPUT);
  pinMode(wpump_pin, OUTPUT);
  pinMode(npump_pin, OUTPUT);
  pinMode(apump_pin, OUTPUT);

  nh.initNode();

  nh.subscribe(led_sub);
  nh.subscribe(wpump_sub);
  nh.subscribe(npump_sub);
  nh.subscribe(apump_sub);

  nh.advertise(temp_pub);
  nh.advertise(humid_pub);
  nh.advertise(light_pub);
  nh.advertise(level_pub);
  nh.advertise(tds_pub);
}

byte temperature = 0;
byte humidity = 0;

void loop(){
  dht11.read(&temperature, &humidity, NULL);

  temp_msg.data = temperature;
  temp_pub.publish(&temp_msg);

  humid_msg.data = humidity;
  humid_pub.publish(&humid_msg);

  light_msg.data = analogRead(light_pin);
  light_pub.publish(&light_msg);

  level_msg.data = analogRead(level_pin);
  level_pub.publish(&level_msg);

  tds_msg.data = analogRead(tds_pin);
  tds_pub.publish(&tds_msg);

  nh.spinOnce();
  delay(1);
}
