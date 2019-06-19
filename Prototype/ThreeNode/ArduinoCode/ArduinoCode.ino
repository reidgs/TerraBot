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

SimpleDHT11 dht11(9);
int led_red_pin = 11;
int led_green_pin = 12;
int led_blue_pin = 13;
int wpump_pin = 9;
int npump_pin = 10;
int apump_pin = 8;



// Functions for Actuators
void led_activate( const std_msgs::Int32& cmd_msg){
  analogWrite(led_red_pin, cmd_msg.data);
  analogWrite(led_green_pin, cmd_msg.data);
  analogWrite(led_blue_pin, cmd_msg.data);//toggle led
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
ros::Subscriber<std_msgs::Int32> led_sub("led_fine", &led_activate);
ros::Subscriber<std_msgs::Bool> wpump_sub("wpump_fine", &wpump_activate);
ros::Subscriber<std_msgs::Bool> npump_sub("npump_fine", &npump_activate);
ros::Subscriber<std_msgs::Bool> apump_sub("apump_fine", &apump_activate);

// Sensors
std_msgs::Int32 temp_msg;
ros::Publisher temp_pub("temperature_raw", &temp_msg);

std_msgs::Int32 humid_msg;
ros::Publisher humid_pub("humidity", &humid_msg);

std_msgs::Int32 light_msg;
ros::Publisher light_pub("light", &light_msg);

void setup(){
  // check this. I misunderstood the pins for the led
  pinMode(led_blue_pin, OUTPUT);
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
}

byte temperature = 0;
byte humidity = 0;

void loop(){
  dht11.read(&temperature, &humidity, NULL);

  temp_msg.data = temperature;
  temp_pub.publish(&temp_msg);

  humid_msg.data = humidity;
  humid_pub.publish(&humid_msg);

  light_msg.data = analogRead(0);
  light_pub.publish(&light_msg);

  nh.spinOnce();
  delay(1);
}
