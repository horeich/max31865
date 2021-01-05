/**
 * @file    max31865_temp.cpp
 * @author  Andreas Reichle (HOREICH UG)
 * 
 */

#include "max31865_temp.hpp"


void max31865_temp::Run()
{
    printf("### example ###\n");

    MAX31865* therm = new MAX31865(); // set default values in mbed_lib.json file

    therm->set_bias(true);
    therm->set_bias(false);
    therm->set_filter_frequency(MAX31865::FILTER_50_HZ);
}