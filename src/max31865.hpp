/**
 * @file    max31865.hpp
 * @author  Andreas Reichle (HOREICH UG)
 * 
 */

#include "drivers/SPI.h"
#include "mbed.h"

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


class MAX31865
{
private:

    struct BitValueMask
    {
        BitValueMask(uint8_t rd, uint8_t wr, uint8_t b, uint8_t bs) : read(rd), write(wr), bits(b), bitshift(bs) {};   
        uint8_t read;
        uint8_t write;
        uint8_t bits;
        uint8_t bitshift;
    };

public:

    enum CONV_MODE
    {
        CONV_MODE_OFF                   = 0x00,
        CONV_MODE_AUTO                  = 0x01,
    };

    enum RTD_MODE
    {
        RTD_MODE_2_WIRE                 = 0x00,
        RTD_MODE_3_WIRE                 = 0x01,
    };

    enum FILTER
    {
        FILTER_60_HZ                    = 0x00,
        FILTER_50_HZ                    = 0x01,
    };

public:

    MAX31865(
        PinName cs = MBED_CONF_MAX31865_CS_PIN,
        PinName clk = MBED_CONF_MAX31865_CLK_PIN,
        PinName miso = MBED_CONF_MAX31865_MISO_PIN,
        PinName mosi = MBED_CONF_MAX31865_MOSI_PIN,
        uint32_t frequency = MBED_CONF_MAX31865_FREQUENCY
    );
    ~MAX31865();

    void set_bias(bool enable);
    bool get_bias();

    void set_conversion_mode(CONV_MODE mode);
    CONV_MODE get_conversion_mode();

    void set_filter_frequency(FILTER filter);
    FILTER get_filter_frequency();

    void set_rtd_mode(RTD_MODE mode);
    RTD_MODE get_rtd_mode();

    short read_temperature();


private:

    uint8_t read_register(char reg);
    void write_register(char reg, char data);

    uint8_t read_value(const BitValueMask& mask);
    void write_value(const BitValueMask& mask, uint8_t value);

private:

    mbed::SPI _spi; 
    mbed::DigitalOut _cs;
    
    static constexpr uint8_t REG_CONFIG             = 0x00;
    static constexpr uint8_t REG_BIAS               = 0x08;
    static constexpr uint8_t REG_MODEAUTO           = 0x04;
    static constexpr uint8_t ADDR_TEMP_MSB          = 0x01;
    static constexpr uint8_t ADDR_TEMP_LSB          = 0x02;

    // CONFIGURATION REGISTER
    const BitValueMask _bias            {0x00, 0x80, 1, 7};
    const BitValueMask _conv_mode       {0x00, 0x80, 1, 6};
    const BitValueMask _one_shot        {0x00, 0x80, 1, 5};
    const BitValueMask _rtd_mode        {0x00, 0x80, 1, 4};
    const BitValueMask _filter_sel      {0x00, 0x80, 1, 0};
    

    // RESISTANCE REGISTER
    //const BitValueMask 

    // 
    // 
    // const BitValueMask _fault_det       {REG_CONFIG, 2, 2};
    // const BitValueMask _fault_status    {REG_CONFIG, 1, 1};
    // const BitValueMask _filter_sel      {REG_CONFIG, 1, 0};

    static constexpr double RTD_A        = 3.9083e-3; // <factor a for Callendar-Van Dusen equation>
    static constexpr double RTD_B        = -5.775e-7; // <factor b for Callendar-Van Dusen equation>

};

