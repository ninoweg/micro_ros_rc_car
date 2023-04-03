#pragma once

namespace rc_car
{
    struct Signal
    {
        double value;
        double pw;
    };

    struct PWM
    {
        unsigned int MIN;
        unsigned int NEUTRAL;
        unsigned int MAX;
    };

    // default
    const unsigned int MIN_PW = 544;      
    const unsigned int NEUTRAL_PW = 1500;
    const unsigned int MAX_PW = 2400;    

    const double TRANSMISSION_FACTOR = 1.0;

    const unsigned int SPEED_PIN = 9;
    const unsigned int STEERING_PIN = 10;
    const unsigned int BATTERY_CELL_PIN_1 = A1;
    const unsigned int BATTERY_CELL_PIN_2 = A2;

    const unsigned int TIMEOUT = 20;
};