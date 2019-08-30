
/*
 * Automated Systems TerraBot Arduino File
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <SimpleDHT.h>

// Internal Values
byte temperature1 = 0;
byte humidity1 = 0;
byte temperature2 = 0;
byte humidity2 = 0;
int lvl = 0;

ros::NodeHandle  nh;

// Sensor Pins
int DHT_pin1 = A8;
int smoist_pin1 = A7;
int light_pin1 = A6;
int DHT_pin2 = A5;
int smoist_pin2 = A4;
int light_pin2 = A3;
int trig_pin = A2;
int echo_pin = A1;
int cur_pin = A0;


// Needed to setup DHT
SimpleDHT22 dht1(DHT_pin1);
SimpleDHT22 dht2(DHT_pin2);

// Actuator pins
int led_pin = 11;
int wpump_pin = 12;
int fan_pin = 13;

// Time dependant variables
float last_update = 0;
long unsigned int last_dht = 0;
float time_now = 0;
int interval = 1000;

// Used to make light/current data more useable
long light_sum1 = 0;
long light_sum2 = 0;
long light_count = 0;

long cur_sum = 0;
long cur_count = 0;


//Frequency Adjustment
void freq_change( const std_msgs::Float32& cmd_msg){
  interval = round(1000.0 / cmd_msg.data);
}

ros::Subscriber<std_msgs::Float32> freq_sub("freq_raw", &freq_change);

// Functions for Actuators
void led_activate( const std_msgs::Int32& cmd_msg){
  analogWrite(led_pin, cmd_msg.data);//toggle led
}

void wpump_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(wpump_pin, cmd_msg.data ? 75 : 0);
}

void fan_activate(const std_msgs::Bool& cmd_msg){
  analogWrite(fan_pin, cmd_msg.data ? 255 : 0);
}

// Actuator subscriptions
ros::Subscriber<std_msgs::Int32> led_sub("led_raw", &led_activate);
ros::Subscriber<std_msgs::Bool> wpump_sub("wpump_raw", &wpump_activate);
ros::Subscriber<std_msgs::Bool> fan_sub("fan_raw", &fan_activate);


// Sensor helpers
float get_level() {
  float duration;
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  duration = float(pulseIn(echo_pin, HIGH, 10000));
  return duration / 5.82;
}

// Sensor publishings

std_msgs::Int32MultiArray humid_msg;
ros::Publisher humid_pub("humid_raw", &humid_msg);

std_msgs::Int32MultiArray temp_msg;
ros::Publisher temp_pub("temp_raw", &temp_msg);

std_msgs::Int32MultiArray light_msg;
ros::Publisher light_pub("light_raw", &light_msg);

std_msgs::Float32 level_msg;
ros::Publisher level_pub("level_raw", &level_msg);

std_msgs::Float32MultiArray cur_msg;
ros::Publisher cur_pub("cur_raw", &cur_msg);

std_msgs::Int32MultiArray smoist_msg;
ros::Publisher smoist_pub("smoist_raw", &smoist_msg);

// Code which is run on the Arduino
void setup(){
  pinMode(led_pin, OUTPUT);
  pinMode(wpump_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  nh.initNode();
  nh.subscribe(freq_sub);

  nh.subscribe(led_sub);
  nh.subscribe(wpump_sub);
  nh.subscribe(fan_sub);

  nh.advertise(temp_pub);
  nh.advertise(humid_pub);
  nh.advertise(light_pub);
  nh.advertise(level_pub);
  nh.advertise(smoist_pub);
  nh.advertise(cur_pub);
}

float to_amp(int analog) {
  return (float(analog) - 512) * .0491;
}
void loop(){
  // Keep track of the amount of light
  if (light_count < 1000) {
    light_count++;
    light_sum1 += analogRead(light_pin1);
    light_sum2 += analogRead(light_pin2);
  }
  // Needed so we can integrate
  if (cur_count < 1000) {
    cur_count++;
    cur_sum += analogRead(cur_pin);
  }

  // updates the reading for temp and humidity
  if(millis() - last_dht > 2500){
      dht1.read(&temperature1, &humidity1, NULL);
      dht2.read(&temperature2, &humidity2, NULL);
      last_dht = millis();
  }

  if(millis() - last_update > interval){
      last_update = millis();

      // Get Temperature
      temp_msg.data_length = 2;
      long int t_array[2];
      t_array[0] = temperature1;
      t_array[1] = temperature2;
      temp_msg.data = t_array;
      temp_pub.publish(&temp_msg);

      // Get Humidity
      humid_msg.data_length = 2;
      long int h_array[2];
      h_array[0] = humidity1;
      h_array[1] = humidity2;
      humid_msg.data = h_array;
      humid_pub.publish(&humid_msg);

      // Get light
      light_msg.data_length = 2;
      long int l_array[2];
      l_array[0] = light_sum1 / light_count;
      l_array[1] = light_sum2 / light_count;
      light_msg.data = l_array;
      light_pub.publish(&light_msg);

      // Reset light values
      light_sum1 = 0;
      light_sum2 = 0;
      light_count = 0;

      // Get the level (complicated enough for own function)
      level_msg.data = get_level();
      level_pub.publish(&level_msg);

      // Get the soil moisture
      smoist_msg.data_length = 2;
      long int s_array[2];
      s_array[0] = analogRead(smoist_pin1);
      s_array[1] = analogRead(smoist_pin2);
      smoist_msg.data = s_array;
      smoist_pub.publish(&smoist_msg);

      // Get current
      cur_msg.data_length = 2;
      float c_array[2];
      c_array[0] = to_amp(analogRead(cur_pin));
      c_array[1] = to_amp(cur_sum / cur_count);
      cur_msg.data = c_array;
      cur_pub.publish(&cur_msg);
      
      // Reset current values
      cur_sum = 0;
      cur_count = 0;
  }
  nh.spinOnce();
}
