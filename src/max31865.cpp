/**
 * @file    max31865.cpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 * TODO:
 * - ifdef mbed_rtos_present
 * - test bias on/off in continuous mode
 */

#include "max31865.hpp"

MAX31865::MAX31865(
    PinName cs,
    PinName clk,
    PinName miso,
    PinName mosi,
    uint32_t frequency,
    RTD_MODE mode,
    uint32_t rm,
    uint32_t rn
) : 
    _spi(mosi, miso, clk),
    _cs(cs, 1),
    _rm(rm), 
    _rn(rn)
{
    _spi.frequency(1000000);
    _spi.format(8, 1); // or _spi.format(8,3)

    set_rtd_mode(mode); // 2/3/4 wires
}

void MAX31865::set_rdy_interrupt(mbed::Callback<void()> cb, PinName rdy)
{
    printf("MAX31865::%s\n", __func__);
    if (cb)
    {
        // Note: it is recommended to use an external pull-up to default to high
        _isr = std::make_unique<mbed::InterruptIn>(rdy, PinMode::PullUp);
        _isr->fall(cb);
    }
    else
    {
        _isr.reset();
    }
}

void MAX31865::wait_async()
{
    _timeout.set(1 << 0);
}

void MAX31865::enable_bias(bool enable)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_bias, (uint8_t)enable);
    if (enable)
    {
        _timeout.clear(1 << 0);
        _async_wait.attach(mbed::callback(this, &MAX31865::wait_async), 74000us);
    }
}

bool MAX31865::is_bias_enabled()
{
    return static_cast<bool>(read_value(_bias));
}

void MAX31865::set_continuous_conversion_mode(FILTER filter)
{
    printf("MAX31865::%s\n", __func__);
    set_conversion_mode(CONV_MODE_AUTO);
    set_filter_frequency(filter);
}

void MAX31865::set_oneshot_conversion_mode()
{
    printf("MAX31865::%s\n", __func__);
    set_conversion_mode(CONV_MODE_OFF);
    enable_bias(false);
}

void MAX31865::set_conversion_mode(CONV_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    if (mode == CONV_MODE_OFF)
    {
        enable_bias(false); // for power saving
    }
    write_value(_conv_mode, mode);
}

MAX31865::CONV_MODE MAX31865::get_conversion_mode()
{
    return static_cast<CONV_MODE>(read_value(_conv_mode));
}

void MAX31865::perform_one_shot_conversion()
{
    write_value(_one_shot, 1);
}



void MAX31865::set_rtd_mode(RTD_MODE mode)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_rtd_mode, mode);
}

MAX31865::RTD_MODE MAX31865::get_rtd_mode()
{
    static_cast<RTD_MODE>(read_value(_rtd_mode));
}

void MAX31865::set_filter_frequency(FILTER filter)
{
    printf("MAX31865::%s\n", __func__);
    write_value(_filter_sel, filter);
}   

MAX31865::FILTER MAX31865::get_filter_frequency()
{
    static_cast<FILTER>(read_value(_filter_sel));
}

float MAX31865::read_temperature()
{
    float Z1, Z2, Z3, Z4, Rt, temp;
    bool _bias_state = true;

    if (!is_bias_enabled())
    {
        enable_bias(true);
        _bias_state = false;
    }
    _timeout.wait_all(1 << 0, osWaitForever, false); 

    uint8_t msb = read_register(ADDR_TEMP_MSB); // low on CS pin initiates a conversion
    uint8_t lsb = read_register(ADDR_TEMP_LSB);

    if (!_bias_state) // return to previous bias state
    {
        enable_bias(false);
    }

    Rt = ((lsb >> 1) | (msb << 7));

    Rt /= 32768;
    Rt *= _rm;

    printf("Rt = %f", Rt);

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

uint8_t MAX31865::read_register(char addr)
{
    addr &= 0x7F; // unset write bit
    _spi.lock();

    _cs.write(0);
    int ret = _spi.write(addr);
    ret = _spi.write(0x00);
    _cs.write(1);
 
    _spi.unlock();
    return ret;
}

void MAX31865::write_register(char addr, char data)
{
    addr &= 0x80; // set write bit
    _spi.lock();

    _cs.write(0);
    int ret = _spi.write(addr); // note that spi_master_write() method can only transmit one char at a time
    ret = _spi.write(data);
    _cs.write(1);

    _spi.unlock();
}

uint8_t MAX31865::read_value(const BitValueMask& mask)
{
    uint8_t value = read_register(mask.read);
    printf("#Read 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(value));
    printf("\n");
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
    reg_value = read_register(mask.read);
    uint8_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value <<= mask.bitshift;
    reg_value &= ~bitmask;
    reg_value |= value;

    write_register(mask.write, reg_value);

    printf("#Wrote 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(reg_value));
    printf("\n");
}