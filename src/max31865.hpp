/**
 * @file    max31865.hpp
 * @author  Andreas Reichle (HOREICH UG) <andreas.reichle@horeich.de>
 */

#ifndef MAX31865_HPP
#define MAX31865_HPP

#include "drivers/SPI.h"
#include "drivers/InterruptIn.h"
#include "drivers/LowPowerTimeout.h"
#include "platform/Callback.h"
#include "rtos/EventFlags.h"
#include "rtos/ThisThread.h"
#include "mbed_error.h"
#include <math.h>
#include <memory>
#include <chrono>

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
        BitValueMask(uint8_t r, uint8_t b, uint8_t bs) : reg(r), bits(b), bitshift(bs) {};   
        uint8_t reg;
        uint8_t bits;
        uint8_t bitshift;
    };

public:

    enum CONV_MODE
    {
        CONV_MODE_OFF                       = 0x00,
        CONV_MODE_AUTO                      = 0x01,
    };  
    
    enum RTD_MODE   
    {   
        RTD_MODE_2_4_WIRE                   = 0x00,
        RTD_MODE_3_WIRE                     = 0x01,
    };  
    
    enum FILTER 
    {   
        FILTER_60_HZ                        = 0x00, // <conversion time around 62.5ms>
        FILTER_50_HZ                        = 0x01, // <conversion time around 52ms>
    };

    enum FAULT
    {
        FAULT_OVER_UNDER_VOLTAGE            = 0x02,
        FAULT_RTDIN_LOW                     = 0x03,
        FAULT_REFIN_LOW                     = 0x04,
        FAULT_REFIN_HIGH                    = 0x05,
        FAULT_RTD_LOW_THRESH                = 0x06,
        FAULT_RTD_HIGH_THRESH               = 0x07,
    };  

    enum FAULT_MODE 
    {   
        FAULT_MODE_OFF                      = 0x00,
        FAULT_MODE_AUTOMATIC_DELAY          = 0x01,
        FAULTMODE_MANUAL_DELAY_CYCLE_1      = 0x02,
        FAULTMODE_MANUAL_DELAY_CYCLE_2      = 0x03,
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
        uint32_t rn = 100
    );

    ~MAX31865() = default;

    /**
     * @brief           Sets callback function called when conversion is ready
     * @param cb        Callback to be called
     * @param rdy       The ready pin @MCU
     * @return          void
     */
    void set_rdy_interrupt(mbed::Callback<void()> cb, PinName rdy = MBED_CONF_MAX31865_RDY_PIN);

    /**
     * @brief           Resets the device registers to power-on-reset (POR) state 
     */
    void soft_reset();
    
    /**
     * @brief   
     */
    void hard_reset();

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


    /**
     * @brief
     */
    void set_oneshot_conversion_mode(FAULT_MODE mode);

    /**
     * @brief               
     * @param filter        50Hz/60Hz noise rejection filter; change BEFORE using continuous conversion mode
     */
    void set_continuous_conversion_mode(FILTER filter);


    CONV_MODE get_conversion_mode();

    void perform_oneshot_conversion();

    void set_filter_frequency(FILTER filter);
    uint8_t get_filter_frequency();

    /**
     * @brief               Sets the device into 2/4 or 3 wire mode
     * @param mode          2/4-wire mode or 3-wire mode
     * @return              void
     */
    void set_rtd_mode(RTD_MODE mode);

    RTD_MODE get_rtd_mode();

    float read_temperature();

    uint8_t read_fault();

    /**
     * @brief               
     */
    void clear_fault();


private:

    /**
     * @brief               Sets the fault mode
     * @param mode          automatic: automatically inserts 100us delays before checking for auflts to allow settle time
     *                      manual delay cycle 1: 
     */
    void set_fault_mode(FAULT_MODE mode);

    /**
     * @brief               Sets conversion mode
     *                      CONV_MODE_AUTO: continuous conversion at 50/60Hz
     *                      CONV_MODE_OFF:
     */
    void set_conversion_mode(CONV_MODE mode);

    void wait_async();

    /**
     *   Possible negative return values:
     *   HAL_OK       = 0x00U
     *   HAL_ERROR    = 0x01U
     *   HAL_BUSY     = 0x02U
     *   HAL_TIMEOUT  = 0x03U
     */
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

    static constexpr uint8_t REG_CONFIG             {0x00};
    static constexpr uint8_t REG_TEMP_HIGH          {0x01}; // <read-only>
    static constexpr uint8_t REG_TEMP_LOW           {0x02}; // <read-only>
    static constexpr uint8_t REG_FAULT_STATUS       {0x07}; // <read-only>

    // CONFIGURATION REGISTER
    const BitValueMask _bias            {REG_CONFIG, 1, 7};
    const BitValueMask _conv_mode       {REG_CONFIG, 1, 6};
    const BitValueMask _one_shot        {REG_CONFIG, 1, 5};
    const BitValueMask _rtd_mode        {REG_CONFIG, 1, 4};
    const BitValueMask _fault_mode      {REG_CONFIG, 2, 2};
    const BitValueMask _fault_clear     {REG_CONFIG, 3, 1}; // <bit 2/3 need to be zero when clearing fault>
    const BitValueMask _filter_sel      {REG_CONFIG, 1, 0};
    
    // FAULT STATUS REGISTER
    
    // RESISTANCE REGISTER
    //const BitValueMask 

    static constexpr double RTD_A        = 3.9083e-3; // <factor a for Callendar-Van Dusen equation>
    static constexpr double RTD_B        = -5.775e-7; // <factor b for Callendar-Van Dusen equation>

};

#endif // MAX31865_HPP