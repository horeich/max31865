/**
 * @file    max31865_oneshot.cpp
 * @author  Andreas Reichle (HOREICH UG)
 * 
 */

#include "max31865_oneshot.hpp"

rtos::EventFlags MAX31865_Oneshot::Rdy_flag;

void MAX31865_Oneshot::Ready()
{
    Rdy_flag.set(1 << 0);
}

void MAX31865_Oneshot::Run()
{
    printf("### MAX31865 one-shot mode example ###\n");

    MAX31865 therm; // set default values in mbed_lib.json file

    // Set RTD mode (3-wire/4-wire)
    // therm.set_rtd_mode(MAX31865::RTD_MODE_2_3_WIRE);

    // Set ready pin interrupt
    therm.set_rdy_interrupt(mbed::callback(Ready));

    // Set 50Hz filter
    therm.set_filter_frequency(MAX31865::FILTER_50_HZ); // conversion using 50Hz filter takes around 62.5ms

    // // Use onehsot conversion mode
    therm.set_oneshot_conversion_mode();
    
    // // Disable bias to save energy (recommended)
    // therm.enable_bias(false);

    // for (int i = 0; i < 20 ; ++i)
    // {
    //     // Wait for ready interrupt to shoot
    //     Rdy_flag.wait_all(1 << 0);

    //     float value = therm.read_temperature();
    //     printf("Temperature [°C]", value);
    // }
}