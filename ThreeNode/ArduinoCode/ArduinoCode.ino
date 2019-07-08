/*
 * Automated Systems Arduino File
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <SimpleDHT.h>

byte temperature = 0;
byte humidity = 0;
int lvl = 0;

ros::NodeHandle  nh;
int DHT_pin = 52;
SimpleDHT22 dht(DHT_pin);

int light_pin = A0;

int l1_pin = A15;
int l2_pin = A14;
int l3_pin = A13;
int lb_pin = A12;

int tds_pin = A1;
int cur_pin = A2;


int led_pin = 9;
int wpump_pin = 12;
int npump_pin = 11;
int apump_pin = 10;
int fan_pin = 8;

float now = millis();
float interval = 100;

long light_sum = 0;
long light_count = 0;

long cur_sum = 0;
long cur_count = 0;

//Frequency Adjustment
void freq_change( const std_msgs::Float32& cmd_msg){
  interval = 1000 / cmd_msg.data;
}

ros::Subscriber<std_msgs::Float32> freq_sub("freq_raw", &freq_change);

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

void fan_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(fan_pin, cmd_msg.data ? 255 : 0);
}

// Actuators
ros::Subscriber<std_msgs::Int32> led_sub("led_raw", &led_activate);
ros::Subscriber<std_msgs::Bool> wpump_sub("wpump_raw", &wpump_activate);
ros::Subscriber<std_msgs::Bool> npump_sub("npump_raw", &npump_activate);
ros::Subscriber<std_msgs::Bool> apump_sub("apump_raw", &apump_activate);
ros::Subscriber<std_msgs::Bool> fan_sub("fan_raw",
&fan_activate);

// Sensor helpers
int get_level() {
  digitalWrite(lb_pin, HIGH);
  lvl =  analogRead(l3_pin) > 100 ? 3 :
         analogRead(l2_pin) > 100 ? 2 :
         analogRead(l1_pin) > 100 ? 1 : 0;
  digitalWrite(lb_pin, LOW);
  return lvl;
}

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

std_msgs::Int32 cur_msg;
ros::Publisher cur_pub("cur_raw", &cur_msg);


void setup(){
  pinMode(led_pin, OUTPUT);
  pinMode(wpump_pin, OUTPUT);
  pinMode(npump_pin, OUTPUT);
  pinMode(apump_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);

  nh.initNode();
  nh.subscribe(freq_sub);

  nh.subscribe(led_sub);
  nh.subscribe(wpump_sub);
  nh.subscribe(npump_sub);
  nh.subscribe(apump_sub);
  nh.subscribe(fan_sub);

  nh.advertise(temp_pub);
  nh.advertise(humid_pub);
  nh.advertise(light_pub);
  nh.advertise(level_pub);
  nh.advertise(tds_pub);
  nh.advertise(cur_pub);
}

void loop(){
  if (light_count < 1000) {
    light_count++;
    light_sum += analogRead(light_pin);
  }

  if (cur_count < 1000) {
    cur_count++;
    cur_sum += analogRead(cur_pin);
  }

  if(millis() - now > interval){
      now = millis();
      dht.read(&temperature, &humidity, NULL);

      temp_msg.data = temperature;
      temp_pub.publish(&temp_msg);

      humid_msg.data = humidity;
      humid_pub.publish(&humid_msg);

      light_msg.data = light_sum / light_count;
      light_sum = 0;
      light_count = 0;
      light_pub.publish(&light_msg);

      level_msg.data = get_level();
      level_pub.publish(&level_msg);

      tds_msg.data = analogRead(tds_pin);
      tds_pub.publish(&tds_msg);

      cur_msg.data = cur_sum / cur_count;
      cur_sum = 0;
      cur_count = 0;
      cur_pub.publish(&cur_msg);
  }
  nh.spinOnce();
}
