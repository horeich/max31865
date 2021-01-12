/**
 * @file    max31865_oneshot.cpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 */

#include "max31865_oneshot.hpp"

using namespace std::chrono_literals;

rtos::EventFlags MAX31865_Oneshot::Rdy_flag;
DigitalOut MAX31865_Oneshot::Led(PA_5); // configured for NUCLEO-L476RG

void MAX31865_Oneshot::Ready()
{
    Led = !Led;
    Rdy_flag.set(1 << 0);
}

void MAX31865_Oneshot::Run()
{
    printf("### MAX31865 one-shot mode example ###\n");

    Led.write(1); // enable led

    MAX31865 therm; // set default values in mbed_lib.json file

    // Resets the device to default values
    therm.soft_reset();

    // Set ready pin interrupt
    therm.set_rdy_interrupt(mbed::callback(Ready));
    
    // Set 50Hz filter (can only be changed in one-shot mode)
    therm.set_filter_frequency(MAX31865::FILTER_50_HZ); // conversion using 50Hz filter takes around 62.5ms

    std::chrono::milliseconds cycle_time = 3000ms;
    for (int i = 0; i < 40; ++i)
    {
        // Use onehsot conversion mode
        therm.set_oneshot_conversion_mode(MAX31865::FAULT_MODE_OFF);
        
        // Wait for ready interrupt to shoot
        Rdy_flag.wait_all_for(1 << 0, cycle_time + 500ms);

        // Read out temperature
        // Make sure to set a timeout in cases the MCU does not shoot an interrupt
        float value = therm.read_temperature();
        printf("Temperature [°C]: %f\n", value);

        // Disable bias to save power/ reduce self-heating
        therm.enable_bias(false);

        // Read fault value
        therm.read_fault();

        // Sleep for a while
        rtos::ThisThread::sleep_for(cycle_time);
    }
}