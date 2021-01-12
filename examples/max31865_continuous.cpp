/**
 * @file    max31865_continuous.cpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 */

#include "max31865_continuous.hpp"

using namespace std::chrono_literals;

rtos::EventFlags MAX31865_Continuous::Rdy_flag;
DigitalOut MAX31865_Continuous::Led(PA_5); // configured for NUCLEO-L476RG

void MAX31865_Continuous::Ready()
{
    Led = !Led;
    Rdy_flag.set(1 << 0);
}

void MAX31865_Continuous::Run()
{
    printf("### MAX31865 one-shot mode example ###\n");

    Led.write(1); // enable led

    MAX31865 therm; // set default values in mbed_lib.json file

    // Resets the device to default values
    therm.soft_reset();

    // Set ready pin interrupt
    therm.set_rdy_interrupt(mbed::callback(Ready));
    
    // Set continuous conversion mode
    therm.set_continuous_conversion_mode(MAX31865::FILTER_50_HZ);

    // Read temperature to reset ready pin
    therm.read_temperature();

    std::chrono::milliseconds cycle_time = 500ms;
    while(1)
    {
        // Wait for ready interrupt to shoot 
        // Make sure to set a timeout in cases the MCU does not shoot an interrupt
        Rdy_flag.wait_all_for(1 << 0, cycle_time + 500ms);

        // Read out the temperature value
        float value = therm.read_temperature();
        printf("Temperature [°C]: %f\n", value);

        // Check last fault
        therm.read_fault();

        rtos::ThisThread::sleep_for(cycle_time); // comment out for full-speed conversion
    }
}