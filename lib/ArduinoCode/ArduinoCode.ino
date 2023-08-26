//#define USE_DHT20
//#define USE_TCA
#define USE_BOTH

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
#include <std_msgs/String.h>
#include <string.h>
#ifdef USE_DHT20
  #include <Wire.h>
  #include <DHT20.h>
#ifdef USE_TCA
  #include <TCA9548.h>
#endif
#else
  #include <dhtnew.h>
  //#include <SimpleDHT.h>
#endif
#include <HX711.h>

// Internal Values
#if 1
int temperature1 = 0;
int humidity1 = 0;
int temperature2 = 0;
int humidity2 = 0;
#else
byte temperature1 = 0;
byte humidity1 = 0;
byte temperature2 = 0;
byte humidity2 = 0;
#endif
int lvl = 0;

ros::NodeHandle  nh;

// Sensor Pins
int light_pin1 = A4;
int smoist_pin1 = A5;
int DHT_pin1 = A6;

int light_pin2 = A8;
int smoist_pin2 = A9;
int DHT_pin2 = A10;

int trig_pin = A2;
int echo_pin = A1;
#ifdef USE_CURRENT
int cur_pin = A0;
#endif

#ifdef USE_TCA
TCA9548 tca(0x70);
#endif

// Set up DHT sensor
#ifdef USE_DHT20
DHT20 dht1;
#ifdef USE_BOTH
DHT20 dht2;
#endif
#else
DHTNEW dht1(DHT_pin1);
#ifdef USE_BOTH
DHTNEW dht2(DHT_pin2);
#endif
#endif

// Actuator pins
//int led_pin = 11;
int led_pin = 10;
int wpump_pin = 12;
int fan_pin = 13;

int weight_sck_pin1 = 2;
int weight_dout_pin1 = 3;
int weight_sck_pin2 = 4;
int weight_dout_pin2 = 5;

HX711 weight1, weight2;
// All of these should be recalibrated on a per-sensor basis
float weight_scale1 = 1;
float weight_offset1 = 0;
float weight_scale2 = -1;
float weight_offset2 = 0;
struct Timing {
  unsigned long next = 0;
  unsigned long period = 1000;
};

// Time dependant variables
long unsigned int last_dht = 0;
unsigned long time_now = 0;
Timing light_timing, temp_timing, humidity_timing;
Timing wlevel_timing, smoist_timing, current_timing;
Timing weight_timing;

// Used to make light/current data more useable
long light_sum1 = 0;
long light_sum2 = 0;
long light_count = 0;

#ifdef USE_CURRENT
long cur_sum = 0;
long cur_count = 0;
#endif

#define SEN_CMP(sensor) (strncmp(cmd_msg.data, sensor, slen) == 0)

//Frequency Adjustment
void freq_change( const std_msgs::String& cmd_msg){
  const char *sep = strchr(cmd_msg.data, '|');
  int slen = sep-cmd_msg.data;
  double freq = atof(sep+1);
  unsigned long period = (freq == 0 ? 99999999 : round(1000.0/freq));
  Timing *timingPtr = (SEN_CMP("light")  ? &light_timing :
		       SEN_CMP("temp")   ? &temp_timing :
		       SEN_CMP("humid")  ? &humidity_timing :
		       SEN_CMP("level")  ? &wlevel_timing :
		       SEN_CMP("smoist") ? &smoist_timing :
		       SEN_CMP("weight") ? &weight_timing :
		       SEN_CMP("cur")    ? &current_timing : NULL);
  if (timingPtr == NULL) printf("Oops\n");
  else {
    timingPtr->period = period;
    timingPtr->next = time_now + period;
  }
}

ros::Subscriber<std_msgs::String> freq_sub("freq_raw", &freq_change);

// Functions for Actuators
void led_activate( const std_msgs::Int32& cmd_msg){
  analogWrite(led_pin, cmd_msg.data);//toggle led
}

void wpump_activate(const std_msgs::Bool& cmd_msg){
//  analogWrite(wpump_pin, cmd_msg.data ? 75 : 0);
  analogWrite(wpump_pin, cmd_msg.data ? 90 : 0);
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

#ifdef USE_CURRENT
std_msgs::Float32MultiArray cur_msg;
ros::Publisher cur_pub("cur_raw", &cur_msg);
#endif

std_msgs::Int32MultiArray smoist_msg;
ros::Publisher smoist_pub("smoist_raw", &smoist_msg);

std_msgs::Float32MultiArray weight_msg;
ros::Publisher weight_pub("weight_raw", &weight_msg);

// Code which is run on the Arduino
void setup(){
  pinMode(led_pin, OUTPUT);
  pinMode(wpump_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

#ifdef USE_TCA
  tca.begin(0);
#endif

  //Wire.setClock(5000);
#ifdef USE_DHT20
  dht1.begin();
#ifdef USE_BOTH
  dht2.begin();
#endif
#else
  dht1.setType(22);
#ifdef USE_BOTH
  dht2.setType(22);
#endif
#endif

  weight1.begin(weight_dout_pin1, weight_sck_pin1);
  weight1.set_scale(weight_scale1);
  weight1.set_offset(weight_offset1);
  weight2.begin(weight_dout_pin2, weight_sck_pin2);
  weight2.set_scale(weight_scale2);
  weight2.set_offset(weight_offset2);

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
  nh.advertise(weight_pub);
#ifdef USE_CURRENT
  nh.advertise(cur_pub);
#endif
}

float to_amp(int analog) {
  return (float(analog) - 512) * .0491;
}
void loop(){
  // Keep track of the amount of light
  time_now = millis();
  if (light_timing.next - time_now < 1000) {
    light_count++;
    light_sum1 += analogRead(light_pin1);
    light_sum2 += analogRead(light_pin2);
  }
#ifdef USE_CURRENT
  // Needed so we can integrate
  if (current_timing.next - time_now < 1000) {
    cur_count++;
    cur_sum += analogRead(cur_pin);
  }
#endif
  // updates the reading for temp and humidity
  if(time_now - last_dht > 2500){
/*
      int status = dht1.read(&temperature1, &humidity1, NULL);
      if (status != SimpleDHTErrSuccess) {
         temperature1 = status; 
         humidity1 = 42;
      }
*/
#ifdef USE_TCA
      tca.selectChannel(1);
#endif
      int status = dht1.read();
#ifdef USE_DHT20
      if (status == DHT20_OK) {
#else
      if (status == DHTLIB_OK) {
#endif
        temperature1 = dht1.getTemperature();
        humidity1 = dht1.getHumidity();
      }
#ifdef USE_BOTH 
#ifdef USE_TCA
      tca.selectChannel(0);
#endif
      status = dht2.read();
#ifdef USE_DHT20
      if (status == DHT20_OK) {
#else
      if (status == DHTLIB_OK) {
#endif
        temperature2 = dht2.getTemperature();
        humidity2 = dht2.getHumidity();
      }
#endif
      last_dht = time_now;
  }

  if(time_now >= temp_timing.next){
      temp_timing.next = time_now + temp_timing.period;

      // Get Temperature
      temp_msg.data_length = 2;
      long int t_array[2];
      t_array[0] = temperature1;
      t_array[1] = temperature2;
      temp_msg.data = t_array;
      temp_pub.publish(&temp_msg);
    }

  if(time_now >= humidity_timing.next){
      humidity_timing.next = time_now + humidity_timing.period;

      // Get Humidity
      humid_msg.data_length = 2;
      long int h_array[2];
      h_array[0] = humidity1;
      h_array[1] = humidity2;
      humid_msg.data = h_array;
      humid_pub.publish(&humid_msg);
  }

  if(time_now >= light_timing.next){
      light_timing.next = time_now + light_timing.period;

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
  }

  if(time_now >= wlevel_timing.next){
      wlevel_timing.next = time_now + wlevel_timing.period;

      // Get the level (complicated enough for own function)
      level_msg.data = get_level();
      // Returns the distance to the water; we want the height of the water.
      // Experimentally, a value of ~180 indicates an empty reservoir
      level_msg.data = 180 - level_msg.data;
      level_pub.publish(&level_msg);
  }

  if(time_now >= smoist_timing.next){
      smoist_timing.next = time_now + smoist_timing.period;

      // Get the soil moisture
      smoist_msg.data_length = 2;
      long int s_array[2];
      s_array[0] = analogRead(smoist_pin1);
      s_array[1] = analogRead(smoist_pin2);
      // Invert the reading, so reading increases as moisture increases
      s_array[0] = 1023 - s_array[0];
      s_array[1] = 1023 - s_array[1];
      smoist_msg.data = s_array;
      smoist_pub.publish(&smoist_msg);
  }

  if(time_now >= weight_timing.next){
      weight_timing.next = time_now + weight_timing.period;

      // Get the weight
      weight_msg.data_length = 2;
      static float w_array[2] = {0,0};
      // The HX711 package sets parameters globally, so rather than
      //   updating the package, need to set parameters for each sensor
      weight1.set_scale(weight_scale1);
      weight1.set_offset(weight_offset1);
      // Sometimes, the weight sensors go offline for a bit - in that case,
      // use the previous value
      if (weight1.is_ready()) w_array[0] = weight1.get_units(1);
      weight2.set_scale(weight_scale2);
      weight2.set_offset(weight_offset2);
      if (weight2.is_ready()) w_array[1] = weight2.get_units(1);
      weight_msg.data = w_array;
      weight_pub.publish(&weight_msg);
  }

#ifdef USE_CURRENT
  if(time_now >= current_timing.next){
      current_timing.next = time_now + current_timing.period;

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
#endif
  nh.spinOnce();
}
