/**
 * @file    max31865.cpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 * TODO:
 * - ifdef mbed_rtos_present
 * - test getter functions
 * - test clear_fault with setting mode normal off before setting the 3 bits
 * FIXME:
 */

#include "max31865.hpp"

using namespace std::chrono_literals;

MAX31865::MAX31865(
    PinName cs,
    PinName clk,
    PinName miso,
    PinName mosi,
    uint32_t frequency,
    uint32_t rm,
    uint32_t rn
) : 

    _spi(mosi, miso, clk),
    _cs(cs, PIN_OUTPUT, PinMode::PullNone, 1),
    _rm(rm), 
    _rn(rn)
{
    _spi.frequency(frequency);
    _spi.format(8, 1); // or _spi.format(8, 3)
}

void MAX31865::disable()
{
    // _analog.reset();
    // DigitalInOut x(_analog_pin, PIN_INPUT, PinMode::PullDown, 0);
    // _enable.input();
    // _enable.write(0);
}

void MAX31865::enable()
{
    // if (!_analog)
    // {
    //     _analog = make_unique<AnalogIn>(_analog_pin);
    //     _enable.output();
    //     _enable.write(1);
    // }
}

void MAX31865::set_rdy_interrupt(mbed::Callback<void()> cb, PinName rdy)
{
    printf("MAX31865::%s\n", __func__);
    if (cb)
    {
        // Note: it is recommended to use an external pull-up to default to high
        _isr = std::make_unique<mbed::InterruptIn>(rdy, PinMode::PullNone);
        _isr->fall(cb);
    }
    else
    {
        _isr.reset();
    }

}

void MAX31865::soft_reset()
{
    printf("MAX31865::%s\n", __func__);
    clear_fault();
    set_conversion_mode(CONV_MODE_OFF);
    enable_bias(false);
    // TODO: set_fault_mode
    set_filter_frequency(FILTER_60_HZ);
}

void MAX31865::wait_async()
{
    //_timeout.set(1 << 0);
}

void MAX31865::enable_bias(bool enable)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_bias, (uint8_t)enable);
    if (enable)
    {
        rtos::ThisThread::sleep_for(74ms);
        // _timeout.clear(1 << 0);
        // _async_wait.attach(mbed::callback(this, &MAX31865::wait_async), 74000us);
    }
}

bool MAX31865::is_bias_enabled()
{
    printf("MAX31865::%s\n", __func__);
    return static_cast<bool>(read_value(_bias));
}

void MAX31865::set_continuous_conversion_mode(FILTER filter)
{
    printf("MAX31865::%s\n", __func__);
    set_conversion_mode(CONV_MODE_OFF); // switch mode to change filter
    set_filter_frequency(filter);
    enable_bias(true); // must be on but not automatically turned on
    set_conversion_mode(CONV_MODE_AUTO);  
}

void MAX31865::set_oneshot_conversion_mode(FAULT_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    set_conversion_mode(CONV_MODE_OFF);
    enable_bias(true); // see p.14 data sheet
    set_fault_mode(mode);
    perform_oneshot_conversion();
}

void MAX31865::set_conversion_mode(CONV_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_conv_mode, mode);
}

MAX31865::CONV_MODE MAX31865::get_conversion_mode()
{
    printf("MAX31865::%s\n", __func__);
    return static_cast<CONV_MODE>(read_value(_conv_mode));
}

void MAX31865::perform_oneshot_conversion()
{
    printf("MAX31865::%s\n", __func__);
    write_value(_one_shot, 1);
}

void MAX31865::set_fault_mode(FAULT_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_fault_mode, mode);
}

uint8_t MAX31865::read_fault()
{
    printf("MAX31865::%s\n", __func__);
    uint8_t fault = read_register(REG_FAULT_STATUS);
    switch (fault)
    {
    case FAULT_OVER_UNDER_VOLTAGE:
        printf("Over/undervoltage fault\n");
        break;
    case FAULT_RTDIN_LOW:
        printf("RTDIN- < 0.85 x V_bias (FORCE- open)");
        break;
    case FAULT_REFIN_LOW:
        printf("REFIN- < 0.85 x V_bias (FORCE- open)");
        break;
    case FAULT_REFIN_HIGH:
        printf("REFIN- > 0.85 x V_bias");
        break;
    case FAULT_RTD_LOW_THRESH:
        printf("RTD low threshold");
        break;
    case FAULT_RTD_HIGH_THRESH:
        printf("RTD high threshold");
        break;
    default:
        printf("No fault\n");
        break;
    }
    return fault;
}

void MAX31865::clear_fault()
{
    printf("MAX31865::%s\n", __func__);
    set_conversion_mode(CONV_MODE_OFF);
    write_value(_fault_clear, 0x01);
}

void MAX31865::set_rtd_mode(RTD_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_rtd_mode, mode);
}

MAX31865::RTD_MODE MAX31865::get_rtd_mode()
{
    return static_cast<RTD_MODE>(read_value(_rtd_mode));
}

void MAX31865::set_filter_frequency(FILTER filter)
{
    printf("MAX31865::%s\n", __func__);
    if (get_conversion_mode() == CONV_MODE_AUTO)
    {
        printf("Cannot change frequency in continuous mode\n");
    }
    else
    {
        write_value(_filter_sel, filter);
    }
}   

uint8_t MAX31865::get_filter_frequency()
{
    return read_value(_filter_sel);
}

float MAX31865::read_temperature()
{
    printf("MAX31865::%s\n", __func__);
    float Z1, Z2, Z3, Z4, Rt, temp;

    uint8_t msb = read_register(REG_TEMP_HIGH); // low on CS pin initiates a conversion
    uint8_t lsb = read_register(REG_TEMP_LOW);

    Rt = ((lsb >> 1) | (msb << 7));

    Rt /= 32768;
    Rt *= _rm;

    printf("Rt = %f\n", Rt);

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / _rn;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;

    if (temp >= 0) // only valid for positive values of the solutions
    {
        return temp;
    }
        
    Rt /= _rn;
    Rt *= 100; // normalize to 100 ohm

    float rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10 * rpoly;

    return temp;
}

uint8_t MAX31865::read_register(char reg)
{
    _spi.lock();
    //_cs.output();
    _cs.write(0);  
    int rc = _spi.write(reg);
    int value = _spi.write(0x00);
    //_cs.input(); 
    _cs.write(1);
    _spi.unlock();

    // Note: The MCU can detect read errors when the MAX31865 does not respond; however,
    // write errors cannot be detected by the MCU
    // Only 0/only 1 may mean that lines continuously on low/high
    if (rc != HAL_OK && rc != 255)
    {
        printf("READ ERROR READ ERROR READ ERROR READ ERROR %d\n", rc);
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_SPI, MBED_ERROR_READ_FAILED), "MAX31865", rc);
    }

    printf("#Read 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(value));
    printf("\n");

    return value;
}

void MAX31865::write_register(char reg, char data)
{
    reg |= 0x80; // set write bit

    _spi.lock();
    //_cs.output();
    _cs.write(0);
    _spi.write(reg); // note that spi_master_write() method can only transmit one char at a time
    //printf("WRITE RC = %d\n", rc);
    _spi.write(data); // expected return value rc = 255
    //printf("WRITE RC = %d\n", rc);
    //_cs.input();
    _cs.write(1);
    _spi.unlock();

    printf("#Wrote 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(data));
    printf("\n");
}

uint8_t MAX31865::read_value(const BitValueMask& mask)
{
    uint8_t value = read_register(mask.reg);
   
    uint8_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value &= bitmask;
    value >>= mask.bitshift;

    printf("Value = %d\n", value);
    return value;
}

void MAX31865::write_value(const BitValueMask& mask, uint8_t value)
{
    uint8_t reg_value{0x00};
    reg_value = read_register(mask.reg);
    uint8_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value <<= mask.bitshift;
    reg_value &= ~bitmask;
    reg_value |= value;

    write_register(mask.reg, reg_value);
}