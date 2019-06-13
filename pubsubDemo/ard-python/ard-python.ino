#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

const int pRes = A0;
const int led_pin = 13;
const int h20sensor = A1;
const int buzzer = 9;

void lightCb(const std_msgs::Int16& lightNeedMsg){
  analogWrite(led_pin, lightNeedMsg.data);
}

void pumpCb(const std_msgs::Bool& pumpMsg){
  int state;
  if (pumpMsg.data){
    tone(buzzer, 1000, 100);
    tone(buzzer, 100, 100);
    tone(buzzer, 500, 100);
  }
  else noTone(buzzer);
  //Serial.println(state);
  //digitalWrite(pump,state);
}
ros::Subscriber<std_msgs::Int16> lghtsub("lightNeedAmt", &lightCb);
ros::Subscriber<std_msgs::Bool> pumpsub("pump", &pumpCb);

std_msgs::Int16 lightHaveMsg;
std_msgs::Int16 waterLevelMsg;
ros::Publisher lghtpub("lightHaveAmt", &lightHaveMsg);
ros::Publisher wtrpub("waterLevelAmt", &waterLevelMsg);

void setup() {
  nh.initNode();
  nh.advertise(wtrpub);
  nh.advertise(lghtpub);
  nh.subscribe(lghtsub);
  nh.subscribe(pumpsub );

  pinMode(led_pin, OUTPUT);
  pinMode(pRes, INPUT);
  pinMode(h20sensor, INPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  lightHaveMsg.data = analogRead(pRes);
  waterLevelMsg.data = analogRead(h20sensor);
  lghtpub.publish(&lightHaveMsg);
  wtrpub.publish(&waterLevelMsg);
  nh.spinOnce();
  delay(1000);
}
