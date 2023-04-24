#pragma once
// arduino
#include <ros.h>
#include <string.h>
#include <Servo.h>
// rc_car
#include "utility.hpp"

namespace rc_car
{
    class Vehicle
    {
    private:
        ros::NodeHandle nh_;
        String name_;
        double length_;             // [m]
        double height_;             // [m]
        double front_track_;        // [m]
        double back_track_;         // [m]
        double weight_;             // [kg]
        double wheel_base_;         // [m]
        double wheel_diameter_;     // [m]
        double top_speed_;          // [m/s]
        double max_steering_angle_; // [rad]
        Signal speed_;              // [m/s]
        Signal steering_angle_;     // [rad]
        Servo servo_speed_;
        Servo servo_steering_;

    public:
        // constructor
        Vehicle(ros::NodeHandle nh, String name, double length, double height, double front_track, double back_track,
                double weight, double wheel_base, double wheel_diameter, double top_speed, double max_steering_angle) : nh_{nh},
                                                                                                                        name_{name},
                                                                                                                        length_{length},
                                                                                                                        height_{height},
                                                                                                                        front_track_{front_track},
                                                                                                                        back_track_{back_track},
                                                                                                                        weight_{weight},
                                                                                                                        wheel_base_{wheel_base},
                                                                                                                        wheel_diameter_{wheel_diameter},
                                                                                                                        top_speed_{top_speed},
                                                                                                                        max_steering_angle_{max_steering_angle},
                                                                                                                        steering_angle_{0.0},
                                                                                                                        speed_{0.0}
        {
        }

        // destructor
        ~Vehicle() = default;

        void attachServos()
        {
            servo_speed_.attach(SPEED_PIN);
            servo_steering_.attach(STEERING_PIN);
        }

        void setSpeed(double speed)
        {
            speed_.value = speed;
            speed_.pw = speedConversion(speed);
            servo_speed_.write(speed_.pw);
        }

        void setSteeringAngle(double steering_angle)
        {
            steering_angle_.value = steering_angle;
            steering_angle_.pw = steeringConversion(steering_angle);
            servo_steering_.write(steering_angle_.pw);
        }

        double speedConversion(double speed)
        {
            unsigned long pw;
            if (abs(speed) <= 1e-3)
                pw = NEUTRAL_PW;
            else if (speed > 0)
            {
                auto delta = MAX_PW - NEUTRAL_PW;
                pw = NEUTRAL_PW + (double)delta * (speed / top_speed_);
            }
            else
            {
                auto delta = NEUTRAL_PW - MIN_PW;
                pw = NEUTRAL_PW + (double)delta * (speed / top_speed_);
            }
            // nh_.loginfo(String(pw).c_str());
            return pw;
        }

        double steeringConversion(double steering_angle)
        {
            return speedConversion(steering_angle);
        }

    };
};