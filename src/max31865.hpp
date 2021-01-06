/**
 * @file    max31865.hpp
 * @author  Andreas Reichle (HOREICH UG)
 * 
 */

#include "drivers/SPI.h"
#include "drivers/InterruptIn.h"
#include "drivers/LowPowerTicker.h"
#include "platform/Callback.h"
#include "rtos/EventFlags.h"
#include "mbed.h"
#include <memory>

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
        RTD_MODE_2_4_WIRE               = 0x00,
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
        uint32_t frequency = MBED_CONF_MAX31865_FREQUENCY,
        RTD_MODE mode = MBED_CONF_MAX31865_RTD_MODE,
        uint32_t rm = MBED_CONF_MAX31865_MATCHING_RESISTANCE,
        uint32_t rn = MBED_CONF_MAX31865_NOMINAL_RESISTANCE
    );

    ~MAX31865() = default;

    void set_rdy_interrupt(mbed::Callback<void()> cb, PinName rdy = MBED_CONF_MAX31865_RDY_PIN);

    /**
     * @brief               Enables or disables BIAS pin output voltage V_bias.
     *                      Note: Disabling the BIAS pin saves energy/ reduces self-heating
     * @param enable        True to enable, false to disable
     * @return              void
     */
    void enable_bias(bool enable);

    /**
     * 
     */
    bool is_bias_enabled();



    void set_oneshot_conversion_mode();

    /**
     * @brief               
     * @param filter        50Hz/60Hz noise rejection filter; change BEFORE using continuous conversion mode
     */
    void set_continuous_conversion_mode(FILTER filter);


    CONV_MODE get_conversion_mode();

    void perform_one_shot_conversion();

    void set_filter_frequency(FILTER filter);
    FILTER get_filter_frequency();

    /**
     * @brief               Sets the device into 2/4 or 3 wire mode
     * @param mode          2/4-wire mode or 3-wire mode
     * @return              void
     */
    void set_rtd_mode(RTD_MODE mode);
    
    RTD_MODE get_rtd_mode();

    float read_temperature();

    uint8_t read_fault();


private:

    /**
     * @brief               Sets conversion mode
     *                      CONV_MODE_AUTO: continuous conversion at 50/60Hz
     *                      CONV_MODE_OFF:
     */
    void set_conversion_mode(CONV_MODE mode);

    void wait_async();

    uint8_t read_register(char reg);
    void write_register(char reg, char data);

    uint8_t read_value(const BitValueMask& mask);
    void write_value(const BitValueMask& mask, uint8_t value);

private:

    mbed::SPI _spi; 
    mbed::DigitalOut _cs;
    uint32_t _rm;                                               // <the matching resistance of the device>
    uint32_t _rn;                                               // <the nominal resistance of the RTD>
    mbed::LowPowerTimeout _async_wait;                          // <timer to wait until filter caps are charged>
    rtos::EventFlags _timeout;                                  // <

    std::unique_ptr<mbed::InterruptIn> _isr;                    // <ready interrupt pin>

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

