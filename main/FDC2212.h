	/*
 * FDC2212.h
 *
 *  Created on: 26 Nov 2017
 *      Author: klemen
 */
// https://www.esp32.com/viewtopic.php?t=1290

#ifndef FDC2212_H_
#define FDC2212_H_

#include <stdint.h>
#include "limits.h"
#include "HardwareSerial.h"
#include <Arduino.h>
#include "Wire.h"
#include <functional>
//#include "driver/i2c.h"
#include "I2Cbus.hpp"

#define FDC2214_I2C_ADDRESS   0x2A
// Address is 0x2A (default) or 0x2B (if ADDR is high)

//bitmasks
#define FDC2212_CH0_UNREADCONV 0x08         //denotes unread CH0 reading in STATUS register

#define FDC2212_CH0_DRDY 			0b10000         	 //denotes data ready in STATUS register
#define FDC2212_CH0_AMPL_LOW 	0b10000000         //denotes amplitude low warning in STATUS register
#define FDC2212_CH0_AMPL_HIGH 0b100000000        //denotes amplitude high warning in STATUS register
#define FDC2212_CH0_WCHD_TO   0b1000000000       //denotes watchdog timeout in STATUS register


//registers
#define FDC2212_MANUFACT_REGADDR            0x7E
#define FDC2212_DEVICE_ID_REGADDR           0x7F
#define FDC2212_MUX_CONFIG_REGADDR          0x1B
#define FDC2212_CONFIG_REGADDR              0x1A
#define FDC2212_SETTLECOUNT_CH0_REGADDR     0x10
#define FDC2212_RCOUNT_CH0_REGADDR          0x08
#define FDC2212_OFFSET_CH0_REGADDR          0x0C
#define FDC2212_CLOCK_DIVIDERS_CH0_REGADDR  0x14
#define FDC2212_STATUS_REGADDR              0x18
#define FDC2212_ERROR_CONFIG              	0x19
#define FDC2212_DATA_CH0_REGADDR            0x00
#define FDC2212_DATA_LSB_CH0_REGADDR        0x01
#define FDC2212_DRIVE_CH0_REGADDR           0x1E
#define FDC2212_RESET_DEV           		0x1C

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           26               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           25               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */

//SemaphoreHandle_t print_mux = NULL;

extern void blink(int i);

class CapacityResponse{
  public:
    uint32_t difference;
    uint32_t timeMs;
    bool status;
};

class FDC2212 {
public:
    typedef std::function< bool (const CapacityResponse& ) > THandlerFunction;
    FDC2212();
    FDC2212(THandlerFunction handler);
    bool begin(void);
    void shouldRead();
    IRAM_ATTR uint32_t getReading();
    static FDC2212 *getInstance();

    bool initialized;
    bool shouldread;
    unsigned long capMax;
    unsigned long capMin;
    float dCap_dT;
    long lastReadingTick;
    unsigned long dT;
    volatile unsigned long readTimeMs;
    volatile unsigned long lastReading;
    volatile float capFast;
    volatile float capSlow;
    volatile uint32_t reading = 0;
    const CapacityResponse& response() { return _response; }

private:
    CapacityResponse _response;
    void loadSettings(void);
    void setGain(void);
	
    double calculateCapacitance(long long fsensor);
    long long calculateFsensor(unsigned long reading);
    //void write8FDC(uint16_t address, uint8_t data);
    void write16FDC(uint8_t address, uint16_t data);
    uint16_t read16FDC(uint8_t address);
    //uint8_t read8FDC(uint16_t address);

    double angle;

    bool isReading;
    float slowFilter;
    float fastFilter;
    float triggerDiff;
	
    static FDC2212* instance;
    i2cbus::I2C myI2C = i2cbus::I2C(I2C_NUM_1);
    THandlerFunction _on_trigger;

    bool status;

    void init();
};

#endif /* FDC2212_H_ */
