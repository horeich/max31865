#include "max31865.hpp"

MAX31865::MAX31865(
    PinName cs,
    PinName clk,
    PinName miso,
    PinName mosi,
    uint32_t frequency
) :
    _spi(PB_15, PB_14, PB_13), // enable on low
    _cs(PC_4, 1)
{
    _spi.frequency(1000000);
    _spi.format(8, 1); // or _spi.format(8,3)
}

MAX31865::~MAX31865()
{
}

void MAX31865::set_bias(bool enable)
{
    write_value(_bias, (uint8_t)enable);
}

bool MAX31865::get_bias()
{
    static_cast<bool>(read_value(_bias));
}

void MAX31865::set_conversion_mode(CONV_MODE mode)
{
    write_value(_conv_mode, mode);
}

MAX31865::CONV_MODE MAX31865::get_conversion_mode()
{
    static_cast<CONV_MODE>(read_value(_conv_mode));
}

void MAX31865::set_filter_frequency(FILTER filter)
{
    write_value(_filter_sel, filter);
}   

MAX31865::FILTER MAX31865::get_filter_frequency()
{
    static_cast<FILTER>(read_value(_filter_sel));
}

void MAX31865::set_rtd_mode(RTD_MODE mode)
{
    write_value(_rtd_mode, mode);
}

MAX31865::RTD_MODE MAX31865::get_rtd_mode()
{
    static_cast<RTD_MODE>(read_value(_rtd_mode));
}

short MAX31865::read_temperature()
{
    //set_bias(true);

    uint8_t msb = read_register(ADDR_TEMP_MSB);
    uint8_t lsb = read_register(ADDR_TEMP_LSB);

    //set_bias(false);

    uint16_t rtd = (lsb >> 1) | (msb << 7);

    rtd = rtd/0x7FFF * MBED_CONF_MAX31865_MATCHING_RESISTOR; // per bit

    float t1 = 

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
    printf("##Read register\n");

    uint8_t reg_value = read_register(mask.read);
    uint8_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    reg_value &= bitmask;
    reg_value >>= mask.bitshift;

    printf("##Value = 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(reg_value));
    printf("\n");

    return reg_value;
}

void MAX31865::write_value(const BitValueMask& mask, uint8_t value)
{
    printf("##Write register\n");

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

    printf("value = %d\n", value);
    printf("##register value = 0b " BYTE_PLACEHOLDER, BYTE_TO_BIN(reg_value));
    printf("\n");

    write_register(mask.write, reg_value);
}