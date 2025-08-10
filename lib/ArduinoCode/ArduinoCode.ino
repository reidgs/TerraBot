/*
   rosserial went away with ros2, and couldn't get microRos to work,
   so doing this with simple serial communication and a ROS2 bridge
*/

//#define USE_DHT20
//#define USE_TCA
#define USE_BOTH

/*
 * Automated Systems TerraBot Arduino File
 */

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
float weight_scale2 = 1;
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

#define SEN_CMP(cmd, sensor) (strncmp(cmd.c_str(), sensor, strlen(sensor)) == 0)

// These macros make it easier to string lines of prints/println
// Do it so don't have to generate a lot of String instances
#define S_PR(val) Serial.print(val)
#define S_PRLN(val) Serial.println(val); delay(100) // try to keep buffer small

//Frequency Adjustment
void freq_change( const String &cmd_msg){
  int sep = cmd_msg.indexOf('|');
  String name = cmd_msg.substring(0, sep);
  float freq = cmd_msg.substring(sep+1).toFloat();
  unsigned long period = (freq == 0 ? 99999999 : round(1000.0/freq));
  Timing *timingPtr = (SEN_CMP(name, "light")  ? &light_timing :
		       SEN_CMP(name, "temp")   ? &temp_timing :
		       SEN_CMP(name, "humid")  ? &humidity_timing :
		       SEN_CMP(name, "level")  ? &wlevel_timing :
		       SEN_CMP(name, "smoist") ? &smoist_timing :
		       SEN_CMP(name, "weight") ? &weight_timing :
		       SEN_CMP(name, "cur")    ? &current_timing : NULL);
  if (timingPtr == NULL) {
    S_PR("# freq_change: Oops: "); S_PRLN(cmd_msg);
  } else {
    S_PR("# freq_change: "); S_PR(name);  S_PR(" "); S_PRLN(period);
    timingPtr->period = period;
    timingPtr->next = time_now + period;
  }
}

// Functions for Actuators
void led_activate( const String& cmd_msg){
  S_PR("# Setting led to : "); S_PRLN(cmd_msg.toInt());
  analogWrite(led_pin, cmd_msg.toInt());//toggle led
}

void wpump_activate(const String& cmd_msg){
  S_PR("# Setting pump to : ");  S_PRLN(cmd_msg.toInt() ? 90 : 0);
//  analogWrite(wpump_pin, cmd_msg ? 75 : 0);
  analogWrite(wpump_pin, cmd_msg.toInt() ? 90 : 0);
}

void fan_activate(const String& cmd_msg){
  S_PR("# Setting fan to : "); S_PRLN(cmd_msg.toInt() ? 255 : 0);
  analogWrite(fan_pin, cmd_msg.toInt() ? 255 : 0);
}

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

// Code which is run on the Arduino
void setup(){
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  S_PRLN("# Comm started");
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
}

float to_amp(int analog) {
  return (float(analog) - 512) * .0491;
}

void actuate(const String &cmd, const String &data) {
  if (cmd == "led") {
    led_activate(data);
  } else if (cmd == "wpump") {
    wpump_activate(data);
  } else if (cmd == "fan") {
    fan_activate(data);
  } else if (cmd == "freq") {
    freq_change(data);
  } else {
    S_PR("# Unknown command received:"); S_PRLN(cmd);
  }
}

void publish1(const String &sensor, long int data) {
  S_PR(sensor); S_PR("|"); S_PRLN(data);
}

void publish2(const String &sensor, long int data_array[2]) {
  S_PR(sensor); S_PR("|"); S_PR(data_array[0]); S_PR(" "); S_PRLN(data_array[1]);
}

void publish2f(const String &sensor, float data_array[2]) {
  S_PR(sensor); S_PR("|"); S_PR(data_array[0]); S_PR(" "); S_PRLN(data_array[1]);
}

void loop(){
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    int sep = msg.indexOf('|');
    String cmd = msg.substring(0, sep);
    String data = msg.substring(sep+1);
    //S_PR("# Received: "); S_PR(cmd); S_PR(" "); S_PRLN(data);
    actuate(cmd, data);
  }

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
      long int t_array[2];
      t_array[0] = temperature1;
      t_array[1] = temperature2;
      publish2("temp", t_array);
    }

  if(time_now >= humidity_timing.next){
      humidity_timing.next = time_now + humidity_timing.period;

      // Get Humidity
      long int h_array[2];
      h_array[0] = humidity1;
      h_array[1] = humidity2;
      publish2("humid", h_array);
  }

  if(time_now >= light_timing.next){
      light_timing.next = time_now + light_timing.period;

      // Get light
      long int l_array[2];
      l_array[0] = light_sum1 / light_count;
      l_array[1] = light_sum2 / light_count;
      publish2("light", l_array);

      // Reset light values
      light_sum1 = 0;
      light_sum2 = 0;
      light_count = 0;
  }

  if(time_now >= wlevel_timing.next){
      wlevel_timing.next = time_now + wlevel_timing.period;

      // Get the level (complicated enough for own function)
      // Returns the distance to the water; we want the height of the water.
      // Experimentally, a value of ~180 indicates an empty reservoir
      int level = 180 - get_level();
      publish1("level", level);
  }

  if(time_now >= smoist_timing.next){
      smoist_timing.next = time_now + smoist_timing.period;

      // Get the soil moisture
      long int s_array[2];
      s_array[0] = analogRead(smoist_pin1);
      s_array[1] = analogRead(smoist_pin2);
      // Invert the reading, so reading increases as moisture increases
      s_array[0] = 1023 - s_array[0];
      s_array[1] = 1023 - s_array[1];
      publish2("smoist", s_array);
  }

  if(time_now >= weight_timing.next){
      weight_timing.next = time_now + weight_timing.period;

      // Get the weight
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
      publish2f("weight", w_array);
  }

#ifdef USE_CURRENT
  if(time_now >= current_timing.next){
      current_timing.next = time_now + current_timing.period;

      // Get current
      cur_msg.data_length = 2;
      float c_array[2];
      c_array[0] = to_amp(analogRead(cur_pin));
      c_array[1] = to_amp(cur_sum / cur_count);
      publish2("cur", c_array);
      
      // Reset current values
      cur_sum = 0;
      cur_count = 0;
  }
#endif
}
