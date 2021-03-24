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

#if MBED_CONF_MAX31865_DEBUG == 1

#define BYTE_PLACEHOLDER "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BIN(byte) \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define debug_print(...)      printf(__VA_ARGS__)
#else
#define debug_print(...)   
#endif

using namespace std::chrono_literals;

MAX31865::MAX31865(
    PinName cs,
    PinName sclk,
    PinName miso,
    PinName mosi,
    uint32_t frequency,
    uint32_t rm,
    uint32_t rn
) : 
    _cs_pin(cs),
    _sclk_pin(sclk),
    _miso_pin(miso),
    _mosi_pin(mosi),
    _spi(nullptr),
    _cs(nullptr),
    _miso(nullptr),
    _mosi(nullptr),
    _sclk(nullptr),
    _isr(nullptr),
    _isr_idle(nullptr),
    _rm(rm), 
    _rn(rn),
    _frequency(frequency)
{

}

void MAX31865::disable()
{
    if (_spi)
    {
        _spi.reset();
        _sclk = std::make_unique<mbed::DigitalInOut>(_sclk_pin, PIN_INPUT, PinMode::PullDown, 0);
        _mosi = std::make_unique<mbed::DigitalInOut>(_mosi_pin, PIN_INPUT, PinMode::PullDown, 0);
        _miso = std::make_unique<mbed::DigitalInOut>(_miso_pin, PIN_INPUT, PinMode::PullDown, 0);
        _cs = std::make_unique<mbed::DigitalInOut>(_cs_pin, PIN_INPUT, PinMode::PullDown, 0); 
    }
}

void MAX31865::enable()
{
    if (!_spi)
    {
        _sclk.reset();
        _mosi.reset();
        _miso.reset();
        _spi = std::make_unique<mbed::SPI>(_mosi_pin, _miso_pin, _sclk_pin);
        _spi->frequency(_frequency);
        _spi->format(8, 1); // or _spi.format(8, 3)
        _cs = std::make_unique<mbed::DigitalInOut>(_cs_pin, PIN_INPUT, PinMode::PullNone, 0);
    }
}

void MAX31865::set_rdy_interrupt(mbed::Callback<void()> callback, PinName rdy, PinMode mode)
{
    debug_print("MAX31865::%s\n", __func__);
    if (callback)
    {
        // Note: it is recommended to use an external pull-up to default to high
        _isr_idle.reset();
        _isr = std::make_unique<mbed::InterruptIn>(rdy, mode);
        _isr->fall(callback);
    }
    else
    {
        _isr.reset();
        _isr_idle = std::make_unique<mbed::DigitalInOut>(rdy, PIN_INPUT, PinMode::PullDown, 0);
    }
}

void MAX31865::soft_reset()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    clear_fault();
    set_conversion_mode(CONV_MODE_OFF);
    enable_bias(false);
    // TODO: set_fault_mode
    set_filter_frequency(FILTER_60_HZ);
}

void MAX31865::enable_bias(bool enable_bias)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    write_value(_bias, (uint8_t)enable_bias);
    if (enable_bias)
    {
        rtos::ThisThread::sleep_for(74ms);
        // _timeout.clear(1 << 0);
        // _async_wait.attach(mbed::callback(this, &MAX31865::wait_async), 74000us);
    }
}

bool MAX31865::is_bias_enabled()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    return static_cast<bool>(read_value(_bias));
}

void MAX31865::set_continuous_conversion_mode(FILTER filter)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    set_conversion_mode(CONV_MODE_OFF); // switch mode to change filter
    set_filter_frequency(filter);
    enable_bias(true); // must be on but not automatically turned on
    set_conversion_mode(CONV_MODE_AUTO);  
}

void MAX31865::set_oneshot_conversion_mode(FAULT_MODE mode)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    set_conversion_mode(CONV_MODE_OFF);
    enable_bias(true); // see p.14 data sheet
    set_fault_mode(mode);
    perform_oneshot_conversion();
}

void MAX31865::set_conversion_mode(CONV_MODE mode)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    write_value(_conv_mode, mode);
}

MAX31865::CONV_MODE MAX31865::get_conversion_mode()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    return static_cast<CONV_MODE>(read_value(_conv_mode));
}

void MAX31865::perform_oneshot_conversion()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    write_value(_one_shot, 1);
}

void MAX31865::set_fault_mode(FAULT_MODE mode)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    write_value(_fault_mode, mode);
}

uint8_t MAX31865::read_fault()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    uint8_t fault = read_register(REG_FAULT_STATUS);
    switch (fault)
    {
    case FAULT_OVER_UNDER_VOLTAGE:
        debug_print("Over/undervoltage fault\n");
        break;
    case FAULT_RTDIN_LOW:
        debug_print("RTDIN- < 0.85 x V_bias (FORCE- open)");
        break;
    case FAULT_REFIN_LOW:
        debug_print("REFIN- < 0.85 x V_bias (FORCE- open)");
        break;
    case FAULT_REFIN_HIGH:
        debug_print("REFIN- > 0.85 x V_bias");
        break;
    case FAULT_RTD_LOW_THRESH:
        debug_print("RTD low threshold");
        break;
    case FAULT_RTD_HIGH_THRESH:
        debug_print("RTD high threshold");
        break;
    default:
        debug_print("No fault\n");
        break;
    }
    return fault;
}

void MAX31865::clear_fault()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    set_conversion_mode(CONV_MODE_OFF);
    write_value(_fault_clear, 0x01);
}

void MAX31865::set_rtd_mode(RTD_MODE mode)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    write_value(_rtd_mode, mode);
}

MAX31865::RTD_MODE MAX31865::get_rtd_mode()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    return static_cast<RTD_MODE>(read_value(_rtd_mode));
}

void MAX31865::set_filter_frequency(FILTER filter)
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    if (get_conversion_mode() == CONV_MODE_AUTO)
    {
        debug_print("Cannot change frequency in continuous mode\n");
    }
    else
    {
        write_value(_filter_sel, filter);
    }
}   

uint8_t MAX31865::get_filter_frequency()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    return read_value(_filter_sel);
}

float MAX31865::read_temperature()
{
    debug_print("MAX31865::%s\n", __func__);
    enable();

    float Z1, Z2, Z3, Z4, Rt, temp;

    uint8_t msb = read_register(REG_TEMP_HIGH); // low on CS pin initiates a conversion
    uint8_t lsb = read_register(REG_TEMP_LOW);

    Rt = ((lsb >> 1) | (msb << 7));

    Rt /= 32768;
    Rt *= _rm;

    debug_print("Rt = %f\n", Rt);

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

/*********************************************************************************
 * PRIVATE MEMBERS
 */

uint8_t MAX31865::read_register(char reg)
{
    _spi->lock();
    _cs->output();  
    int rc = _spi->write(reg);
    int value = _spi->write(0x00);
    _cs->input();
    _spi->unlock();

    // Note: The MCU can detect read errors when the MAX31865 does not respond; however,
    // write errors cannot be detected by the MCU
    // Only 0/only 1 may mean that lines continuously on low/high
    if (rc != HAL_OK && rc != 255)
    {
        debug_print("READ ERROR READ ERROR READ ERROR READ ERROR %d\n", rc);
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_SPI, MBED_ERROR_READ_FAILED), "MAX31865", rc);
    }

    debug_print("#Read 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(value));
    debug_print("\n");

    return value;
}

void MAX31865::write_register(char reg, char data)
{
    reg |= 0x80; // set write bit

    _spi->lock();
    _cs->output();
    _spi->write(reg); // note that spi_master_write() method can only transmit one char at a time
    //debug_print("WRITE RC = %d\n", rc);
    _spi->write(data); // expected return value rc = 255
    //debug_print("WRITE RC = %d\n", rc);
    _cs->input();
    _spi->unlock();

    debug_print("#Wrote 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(data));
    debug_print("\n");
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

    debug_print("Value = %d\n", value);
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