#ifndef __PICO_DPS310__
#define __PICO_DPS310__

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class DPS310{
    public:
        /* TODO:Error handle */
        enum class RESULT{
            OK = 0x01,
            FAIL = 0x02,
        };
        
        /* SETTINGS */
        enum class PRESSURE_MEAS_RATE{
            RATE_1Hz   = 0x00,
            RATE_2Hz   = 0x01,
            RATE_4Hz   = 0x02,
            RATE_8Hz   = 0x03,
            RATE_16Hz  = 0x04,
            RATE_32Hz  = 0x05,
            RATE_64Hz  = 0x06,
            RATE_128Hz = 0x07
        };

        enum class PRESSURE_OVER_SAMPLING_RATE{
            RATE_1Hz   = 0x00,
            RATE_2Hz   = 0x01,
            RATE_4Hz   = 0x02,
            RATE_8Hz   = 0x03,
            RATE_16Hz  = 0x04,
            RATE_32Hz  = 0x05,
            RATE_64Hz  = 0x06,
            RATE_128Hz = 0x07            
        };

        enum class TEMPERATURE_MEAS_EXTERNAL{
            INTERNAL = 0x00,
            EXTERNAL = 0x01
        };

        enum class TEMPERATURE_MEAS_RATE{
            RATE_1Hz   = 0x00,
            RATE_2Hz   = 0x01,
            RATE_4Hz   = 0x02,
            RATE_8Hz   = 0x03,
            RATE_16Hz  = 0x04,
            RATE_32Hz  = 0x05,
            RATE_64Hz  = 0x06,
            RATE_128Hz = 0x07                 
        };

        enum class TEMPERATURE_OVER_SAMPLING_RATE{
            RATE_1Hz   = 0x00,
            RATE_2Hz   = 0x01,
            RATE_4Hz   = 0x02,
            RATE_8Hz   = 0x03,
            RATE_16Hz  = 0x04,
            RATE_32Hz  = 0x05,
            RATE_64Hz  = 0x06,
            RATE_128Hz = 0x07            
        };

        enum class MEAS_MODE{
            IDLE = 0x00,
            ONCE_PRESSURE_MEAS = 0x01,
            ONCE_TEMPERATURE_MEAS = 0x02,
            CONTINUOUS_PRESSURE_MEAS = 0x05,
            CONTINUOUS_TEMPERATURE_MEAS = 0x06,
            CONTINUOUS_PRESSURE_AND_TEMPERATURE_MEAS = 0x07
        };

        enum class INTERRUPT_ACTIVE_LEVEL{
            ACTIVE_LOW = 0x00,
            ACTIVE_HIGH = 0x01
        };

        enum class INTERRUPUT_FIFO_FULL{
            DISABLE = 0x00,
            ENABLE = 0x01
        };

        enum class INTERRUPT_TEMPERATURE_READY{
            DISABLE = 0x00,
            ENABLE = 0x01
        };

        enum class INTERRUPT_PRESSURE_READY{
            DISABLE = 0x00,
            ENABLE = 0x01
        };

        enum class TEMPERATURE_BITSHIFT{
            DISABLE = 0x00,
            ENABLE = 0x01
        };

        enum class PRESSURE_BITSHIFT{
            DISABLE = 0x00,
            ENABLE = 0x01
        };

        enum class FIFO_EN{
            DISABLE = 0x00,
            ENABLE = 0x01           
        };

        enum class SPI_MODE{
            FOUR_WIRE = 0x00,
            THREE_WIRE = 0x01
        };

        /* Function */
        DPS310(i2c_inst_t *i2c,uint sda_pin,uint scl_pin ,uint baudrate);
        ~DPS310();
        void measurement();
        float getPressure();
        float getTemperature();
        void pressureConfiguration(
            PRESSURE_MEAS_RATE pm_rate = PRESSURE_MEAS_RATE::RATE_4Hz,
            PRESSURE_OVER_SAMPLING_RATE pm_prc = PRESSURE_OVER_SAMPLING_RATE::RATE_8Hz);
        void temperatureConfiguration(
            TEMPERATURE_MEAS_EXTERNAL tmp_ext = TEMPERATURE_MEAS_EXTERNAL::EXTERNAL,
            TEMPERATURE_MEAS_RATE tmp_rate = TEMPERATURE_MEAS_RATE::RATE_4Hz,
            TEMPERATURE_OVER_SAMPLING_RATE tmp_prc = TEMPERATURE_OVER_SAMPLING_RATE::RATE_8Hz);
        void measureConfiguration(MEAS_MODE meas_ctrl = MEAS_MODE::CONTINUOUS_PRESSURE_AND_TEMPERATURE_MEAS);
        void interruptFifoConfiguration(
            SPI_MODE spi_mode = SPI_MODE::FOUR_WIRE,
            FIFO_EN fifo_en = FIFO_EN::DISABLE,
            PRESSURE_BITSHIFT p_shift = PRESSURE_BITSHIFT::DISABLE,
            TEMPERATURE_BITSHIFT t_shift = TEMPERATURE_BITSHIFT::DISABLE,
            INTERRUPT_PRESSURE_READY int_prs = INTERRUPT_PRESSURE_READY::DISABLE,
            INTERRUPT_TEMPERATURE_READY int_tmp = INTERRUPT_TEMPERATURE_READY::DISABLE,
            INTERRUPUT_FIFO_FULL int_fifo = INTERRUPUT_FIFO_FULL::DISABLE,
            INTERRUPT_ACTIVE_LEVEL int_hl = INTERRUPT_ACTIVE_LEVEL::ACTIVE_LOW);

    private:
        /* Variable */
        float pressure;
        float temperature;
        int32_t pressure_raw;
        int32_t temperature_raw;       

        struct COEFFICIENT {
            int32_t c00,c10,c0,c1,c01,c11,c20,c21,c30;
            int32_t kT,kP;
        } coefficient;

        /* Function */
        void readCoefficient();
        uint8_t readProductId();
        void setkP();
        void setkT();
        int32_t twosComplement(int32_t coef,uint8_t digit);

        /* REGISTER */
        enum REG : uint8_t{
            PSR_B2     = 0x00, /* Pressure Out put Register */
            PSR_B1     = 0x01,
            PSR_B0     = 0x02,
            TMP_B2     = 0x03, /* Temperature Out put Register */
            TMP_B1     = 0x04,
            TMP_B0     = 0x05,
            PRS_CFG    = 0x06, /* Pressure Configuration */
            TMP_CFG    = 0x07, /* Temperature Configuration */
            MEAS_CFG   = 0x08, /* Sensor Operating Mode and Status */
            CFG_RES    = 0x09, /* Interrupt and FIFO COnfiguration */
            INT_STS    = 0x0A, /* Interrupt Status */
            FIFO_STS   = 0x0B, /* FIFO Status */
            RESET      = 0x0C, /* Soft Reset and FIFO flush */
            PRODUCT_ID = 0x0D, /* Product and Revision ID */
            COEF       = 0x10, /* Calibration Coefficient */
            COEF_SRCE  = 0x28, /* COefficient Source */
            ADDRESS    = 0x77  /* Slave Address */
        };

        #pragma pack(push, 1)
      
        /* REGISTER CONTENT */
        union PRESSURE_CONFIGURATION{
            struct {
                PRESSURE_OVER_SAMPLING_RATE pm_prc : 4;
                PRESSURE_MEAS_RATE pm_rate         : 3;
                bool reserve                       : 1;
            } contents;
            uint8_t byte;
        } press_config;

        union TEMPERATURE_CONFIGURATION{
            struct {
                TEMPERATURE_OVER_SAMPLING_RATE tmp_prc : 4;
                TEMPERATURE_MEAS_RATE tmp_rate         : 3;
                TEMPERATURE_MEAS_EXTERNAL tmp_ext      : 1;
            } contents;
            uint8_t byte;
        } temp_config;

        union MEASURE_CONFIGURATION{
            struct {
                MEAS_MODE meas_ctrl : 3;
                uint8_t reserve     : 5;
            } contents;
            uint8_t byte;
        } meas_config;

        union INTERRUPT_FIFO_CONFIGURATION{
            struct {
                SPI_MODE spi_mode                   : 1;
                FIFO_EN fifo_en                     : 1;
                PRESSURE_BITSHIFT p_shift           : 1;
                TEMPERATURE_BITSHIFT t_shift        : 1;
                INTERRUPT_PRESSURE_READY int_prs    : 1;
                INTERRUPT_TEMPERATURE_READY int_tmp : 1;
                INTERRUPUT_FIFO_FULL int_fifo       : 1;
                INTERRUPT_ACTIVE_LEVEL int_hl       : 1;
            } contents;
            uint8_t byte;
        } int_fifo_config;

        #pragma pack(pop)
};

#endif