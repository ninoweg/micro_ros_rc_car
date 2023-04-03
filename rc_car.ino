// arduino
#include <Arduino.h>
#include <ros.h>
// ros
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/BatteryState.h>
// rc_car
#include "include/utility.hpp"
#include "include/vehicle.hpp"

using namespace rc_car;

ros::NodeHandle nh;
bool wd_ackermann;

// https://traxxas.com/products/models/electric/58034-61slash?t=specs
Vehicle traxxas("traxxas_slash_2wd", 0.578, 0.214, 0.296, 0.296, 2.16, 0.335, 0.05588, 13.5, M_PI/8);

sensor_msgs::BatteryState battery_state;
ros::Publisher pub_battery_state("/rc_car/battery_state", &battery_state);

void callbackAckermannDriveCmds(const ackermann_msgs::AckermannDrive& msg)
{
  traxxas.setSpeed(msg.speed); // m/s
  traxxas.setSteeringAngle(msg.steering_angle); // rad
  wd_ackermann = true;
}
ros::Subscriber<ackermann_msgs::AckermannDrive> sub_ackermann("ackermann_drive_cmds", callbackAckermannDriveCmds);

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
  static unsigned int last{millis()};
  auto dt = millis() - last;
  if (dt > TIMEOUT)
  {
    // watchdog timer ackermann
    if(!wd_ackermann)
    {
      traxxas.setSpeed(NEUTRAL_PW); // m/s
      traxxas.setSteeringAngle(NEUTRAL_PW); // rad
    }
    wd_ackermann = false;

    // check inputs
    readInputs();
  }

}

void setup()
{
  nh.initNode(); 
  traxxas.attachServos();
}

void loop()
{
  nh.spinOnce();
  checkTimer();
  delay(1);
}