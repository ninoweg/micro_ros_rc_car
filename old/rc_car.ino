// arduino
#include <Arduino.h>
#include <ros.h>
// ros
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
// rc_car
#include "include/utility.hpp"
#include "include/vehicle.hpp"

using namespace rc_car;

ros::NodeHandle nh;
bool wd_ackermann;

// https://traxxas.com/products/models/electric/58034-61slash?t=specs
Vehicle traxxas(nh, "traxxas_slash_2wd", 0.578, 0.214, 0.296, 0.296, 2.16, 0.335, 0.05588, 13.5, M_PI/8);

sensor_msgs::BatteryState battery_state;
ros::Publisher pub_battery_state("/rc_car/battery_state", &battery_state);

void callbackAckermannDriveCmds(const geometry_msgs::Twist& msg)
{
  traxxas.setSpeed(msg.linear.x); // m/s
  traxxas.setSteeringAngle(-msg.angular.z); // rad
  wd_ackermann = true;
}
ros::Subscriber<geometry_msgs::Twist> sub_ackermann("drive_cmds", callbackAckermannDriveCmds);

void readInputs()
{
  static unsigned int seq{0};
  auto cell_voltage_1 = analogRead(BATTERY_CELL_PIN_1) * (5.0 / 1023.0);
  auto cell_voltage_2 = analogRead(BATTERY_CELL_PIN_2) * (5.0 / 1023.0);
  float cell_volages[] = {cell_voltage_1, cell_voltage_2};

  battery_state.header.frame_id = "rc_car";
  battery_state.header.seq = seq++;
  battery_state.header.stamp = nh.now();
  battery_state.cell_voltage = cell_volages;
  battery_state.voltage = cell_voltage_1 + cell_voltage_2;

  pub_battery_state.publish(&battery_state);
}

void checkTimer()
{
  static unsigned long last{millis()};
  auto dt = millis() - last;
  
  if (dt > TIMEOUT)
  {
    // watchdog timer ackermann
    if(!wd_ackermann)
    {
      traxxas.setSpeed(0.0); // m/s
      // nh.logwarn("watchdog timeout for drive cmds");
    }
    wd_ackermann = false;

    // check inputs
    readInputs();

    // update last
    last = millis();
  }
}

void setup()
{
  nh.initNode(); 
  traxxas.attachServos();
  nh.advertise(pub_battery_state);
  nh.subscribe(sub_ackermann);
  pinMode(A0, INPUT);
  pinMode(2, INPUT);
}

void loop()
{
  nh.spinOnce();
  checkTimer();
  // delay(20);
  // auto a = analogRead(A0);
  // nh.loginfo(String(a).c_str());
  auto d = digitalRead(2);
  if (d == LOW)
    nh.loginfo(String(d).c_str());
}