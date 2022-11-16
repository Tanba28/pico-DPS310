#include "DPS310.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"

/* Constructor */
DPS310::DPS310(i2c_inst_t *i2c,uint sda_pin,uint scl_pin ,uint baudrate){
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    //settings
    DPS310::readCoefficient();
    DPS310::pressureConfiguration();
    DPS310::temperatureConfiguration();
    DPS310::interruptFifoConfiguration();
    DPS310::measureConfiguration(); 
}

/* Public Function */
void DPS310::measurement(){
    uint8_t var;
    uint8_t buf[6];
    
    var = DPS310::REG::PSR_B2;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,&var,1,true);
    i2c_read_blocking(i2c1,DPS310::REG::ADDRESS,buf,6,false);

    this->pressure_raw    = twosComplement(((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2],24);
    this->temperature_raw = twosComplement(((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5],24);

    this->pressure = (float)this->pressure_raw/this->coefficient.kP;

    this->temperature = (float)this->temperature_raw/this->coefficient.kT;
   
    this->pressure = this->coefficient.c00 +
                     this->pressure * (this->coefficient.c10 +
                                           this->pressure * (this->coefficient.c20 + this->pressure * this->coefficient.c30)) +
                     this->temperature * this->coefficient.c01 +
                     this->temperature * this->pressure * (this->coefficient.c11 + this->pressure * this->coefficient.c21);
    this->temperature = 0.5 * this->coefficient.c0 + this->coefficient.c1 * this->temperature;
}

float DPS310::getPressure(){
    return this->pressure;
}

float DPS310::getTemperature(){
    return this->temperature;
}

void DPS310::pressureConfiguration(
    PRESSURE_MEAS_RATE pm_rate,
    PRESSURE_OVER_SAMPLING_RATE pm_prc){

    uint8_t var[2];

    this->press_config.contents.pm_prc = pm_prc;
    this->press_config.contents.pm_rate = pm_rate;
    
    
    var[0] = DPS310::REG::PRS_CFG;
    var[1] = this->press_config.byte;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,var,2,false);

    DPS310::setkP();    
}

void DPS310::temperatureConfiguration(
    TEMPERATURE_MEAS_EXTERNAL tmp_ext,
    TEMPERATURE_MEAS_RATE tmp_rate,
    TEMPERATURE_OVER_SAMPLING_RATE tmp_prc){

    uint8_t var[2];

    this->temp_config.contents.tmp_prc = tmp_prc;
    this->temp_config.contents.tmp_rate = tmp_rate;
    this->temp_config.contents.tmp_ext = tmp_ext;
    
    var[0] = DPS310::REG::TMP_CFG;
    var[1] = this->temp_config.byte;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,var,2,false);

    DPS310::setkT();
}

void DPS310::measureConfiguration(MEAS_MODE meas_ctrl){
    uint8_t var[2];

    this->meas_config.contents.meas_ctrl = meas_ctrl;
    
    var[0] = DPS310::REG::MEAS_CFG;
    var[1] = this->meas_config.byte;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,var,2,false);
}

void DPS310::interruptFifoConfiguration(SPI_MODE spi_mode,
    FIFO_EN fifo_en,
    PRESSURE_BITSHIFT p_shift,
    TEMPERATURE_BITSHIFT t_shift,
    INTERRUPT_PRESSURE_READY int_prs,
    INTERRUPT_TEMPERATURE_READY int_tmp,
    INTERRUPUT_FIFO_FULL int_fifo,
    INTERRUPT_ACTIVE_LEVEL int_hl){
    uint8_t var[2];

    this->int_fifo_config.contents.spi_mode = spi_mode;
    this->int_fifo_config.contents.fifo_en = fifo_en;
    this->int_fifo_config.contents.p_shift = p_shift;
    this->int_fifo_config.contents.t_shift = t_shift;
    this->int_fifo_config.contents.int_prs = int_prs;
    this->int_fifo_config.contents.int_tmp = int_tmp;
    this->int_fifo_config.contents.int_fifo = int_fifo;
    this->int_fifo_config.contents.int_hl = int_hl;
    
    var[0] = DPS310::REG::CFG_RES;
    var[1] = this->int_fifo_config.byte;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,var,2,false);    

}

/* Private Function*/
void DPS310::readCoefficient(){
    uint8_t var;
    uint8_t buf[18];

    uint32_t mask;

    var = DPS310::REG::COEF;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,&var,1,true);
    i2c_read_blocking(i2c1,DPS310::REG::ADDRESS,buf,18,false);

    mask = 0xFFF;
    this->coefficient.c0  = DPS310::twosComplement((((uint32_t)buf[0] << 4) | ((uint32_t)buf[1] >> 4)) & mask,12);
    this->coefficient.c1  = DPS310::twosComplement((((uint32_t)buf[1] << 8) | ((uint32_t)buf[2]     )) & mask,12);

    mask = 0xFFFFF;
    this->coefficient.c00 = DPS310::twosComplement((((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4)) & mask,20);
    this->coefficient.c10 = DPS310::twosComplement((((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 8) | ((uint32_t)buf[7]     )) & mask,20);

    this->coefficient.c01 = DPS310::twosComplement((((uint32_t)buf[8]  << 8) | ((uint32_t)buf[9])),16);
    this->coefficient.c11 = DPS310::twosComplement((((uint32_t)buf[10] << 8) | ((uint32_t)buf[11])),16);
    this->coefficient.c20 = DPS310::twosComplement((((uint32_t)buf[12] << 8) | ((uint32_t)buf[13])),16);
    this->coefficient.c21 = DPS310::twosComplement((((uint32_t)buf[14] << 8) | ((uint32_t)buf[15])),16);
    this->coefficient.c30 = DPS310::twosComplement((((uint32_t)buf[16] << 8) | ((uint32_t)buf[17])),16);
}

uint8_t DPS310::readProductId(){
    uint8_t var;
    uint8_t buf;
    
    var = DPS310::REG::PRODUCT_ID;
    i2c_write_blocking(i2c1,DPS310::REG::ADDRESS,&var,1,true);
    i2c_read_blocking(i2c1,DPS310::REG::ADDRESS,&buf,1,false);

    return buf;
}

void DPS310::setkP(){
    switch (this->press_config.contents.pm_prc)
    {
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_1Hz:
        this->coefficient.kP = 524288;
        break;
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_2Hz:
        this->coefficient.kP = 1572864;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_4Hz:
        this->coefficient.kP = 3670016;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_8Hz:
        this->coefficient.kP = 7864320;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_16Hz:
        this->coefficient.kP = 253952;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_32Hz:
        this->coefficient.kP = 516096;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_64Hz:
        this->coefficient.kP = 1040384;
        break;   
    case DPS310::PRESSURE_OVER_SAMPLING_RATE::RATE_128Hz:
        this->coefficient.kP = 2088960;
        break;   
    }
}
void DPS310::setkT(){
    switch (this->temp_config.contents.tmp_prc)
    {
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_1Hz:
        this->coefficient.kT = 524288;
        break;
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_2Hz:
        this->coefficient.kT = 1572864;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_4Hz:
        this->coefficient.kT = 3670016;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_8Hz:
        this->coefficient.kT = 7864320;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_16Hz:
        this->coefficient.kT = 253952;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_32Hz:
        this->coefficient.kT = 516096;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_64Hz:
        this->coefficient.kT = 1040384;
        break;   
    case DPS310::TEMPERATURE_OVER_SAMPLING_RATE::RATE_128Hz:
        this->coefficient.kT = 2088960;
        break;   
    }
}

int32_t DPS310::twosComplement(int32_t coef,uint8_t digit){
    if(coef & ((uint32_t)1 << (digit-1))){
        coef -= (uint32_t)1 << digit;
    }
    return coef;
}

