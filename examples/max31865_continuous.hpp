/**
 * @file    max31865_continuous.hpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 */

#ifndef MAX31865_CONTINUOUS_HPP
#define MAX31865_CONTINUOUS_HPP

#include "../src/max31865.hpp"
#include "mbed.h"
#include "rtos.h"

class MAX31865_Continuous
{
public:
    MAX31865_Continuous() = default;
    ~MAX31865_Continuous() = default;

    static void Run();

private:  
    static void Ready();

private:
    static rtos::EventFlags Rdy_flag;
    static mbed::DigitalOut Led;
};

#endif // MAX31865_CONTINUOUS_HPP