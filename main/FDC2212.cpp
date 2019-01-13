/*
 * FDC2212.cpp
 *
 *  Created on: 26 Nov 2017
 *      Author: klemen
 */

#include "FDC2212.h"

FDC2212* FDC2212::instance = 0;

FDC2212* FDC2212::getInstance() {
	if (!instance) {
		instance = new FDC2212();
	}
	return instance;
}

esp_err_t ma_read_uint16t(uint8_t address, uint8_t *rbuf) {
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	ret = i2c_master_start(cmd);
	if (ret != ESP_OK)
		Serial.println("err r1");
	ret = i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS << 1) | WRITE_BIT,
			ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err r2");
	ret = i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err r3");
	ret = i2c_master_start(cmd);
	if (ret != ESP_OK)
		Serial.println("err r4");
	ret = i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS << 1) | READ_BIT,
			ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err r5");

	ret = i2c_master_read_byte(cmd, rbuf++, I2C_MASTER_ACK);
	if (ret != ESP_OK)
		Serial.println("err r6");
	ret = i2c_master_read_byte(cmd, rbuf++, I2C_MASTER_NACK);
	if (ret != ESP_OK)
		Serial.println("err r7");

	ret = i2c_master_stop(cmd);
	if (ret != ESP_OK)
		Serial.println("err r8");

	ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 20 / portTICK_RATE_MS);
	if (ret == ESP_FAIL)
		Serial.println("err r9 - ESP_FAIL");
	else if (ret == ESP_ERR_INVALID_STATE)
		Serial.println("err r9 - ESP_ERR_INVALID_STATE");
	else if (ret == ESP_ERR_TIMEOUT)
		Serial.println("err r9 - ESP_ERR_TIMEOUT");

	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		printf("i2c Error read - readback\n");
		return ESP_FAIL;
	}
	return ret;
}

/*
 esp_err_t ma_write_uint8t(uint8_t address, uint8_t value)
 {
 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS<<1) | WRITE_BIT , ACK_CHECK_EN);
 i2c_master_write_byte(cmd, address, I2C_MASTER_NACK);
 i2c_master_write_byte(cmd, value, I2C_MASTER_ACK);
 i2c_master_stop(cmd);
 esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 20 / portTICK_RATE_MS);
 i2c_cmd_link_delete(cmd);
 if (ret == ESP_FAIL) {
 printf("ESP_I2C_WRITE ERROR : %d\n",ret);
 return ret;
 }
 return ESP_OK;
 }
 */

esp_err_t ma_write_uint16t(uint8_t address, uint16_t value) {
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ret = i2c_master_start(cmd);
	if (ret != ESP_OK)
		Serial.println("err w1");

	ret = i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS << 1) | WRITE_BIT,
			ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err w2");
	ret = i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err w3");
	ret = i2c_master_write_byte(cmd, value >> 8, ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err w4");
	ret = i2c_master_write_byte(cmd, value << 8, ACK_CHECK_EN);
	if (ret != ESP_OK)
		Serial.println("err w5");
	ret = i2c_master_stop(cmd);
	if (ret != ESP_OK)
		Serial.println("err w6");

	ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 20 / portTICK_RATE_MS);
	if (ret == ESP_FAIL)
		Serial.println("err w7 - ESP_FAIL");
	else if (ret == ESP_ERR_INVALID_STATE)
		Serial.println("err w7 - ESP_ERR_INVALID_STATE");
	else if (ret == ESP_ERR_TIMEOUT)
		Serial.println("err w7 - ESP_ERR_TIMEOUT");
	i2c_cmd_link_delete(cmd);
	return ESP_OK;
}

void FDC2212::init() {
	initialized = false;
	capMin = ULONG_MAX;
	capMax = 0;
	isReading = false;
	shouldread = false;
	lastReadingTick = 0;
	lastReading = 0;
	dCap_dT = 0;
	angle = 0;
	slowFilter = 0.008;
	fastFilter = 0.2;
	triggerDiff = 10000.0;
	_response.status = false;
	capSlow = 0.0;
	capFast = 0.0;
}

FDC2212::FDC2212() {
	this->init();
}

FDC2212::FDC2212(THandlerFunction fn) {
	this->init();
	_on_trigger = fn;
}

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool FDC2212::begin(void) {
	//esp_err_t ret = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
	ESP_ERROR_CHECK(this->myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, 100000));
	myI2C.setTimeout(10);
	myI2C.scanner();

	Serial.println("i2cInit");
	//_i2c = i2cInit(1,21,22,100000);

	vTaskDelay(20 / portTICK_PERIOD_MS);
	if (&(this->myI2C) != NULL) {
		/*
		 uint16_t manufacturer = read16FDC(FDC2212_MANUFACT_REGADDR);
		 Serial.printf("i2c manufacturer %x\n", manufacturer);
		 if (manufacturer != 0x5449) {
		 Serial.printf("Wrong i2c device manufacturer %x\n", manufacturer);
		 //two valid device ids for FDC2214 0x3054 and 0x3055
		 return false;
		 }
		 */
		uint16_t manufacturer = read16FDC(FDC2212_MANUFACT_REGADDR);
		Serial.printf("i2c manufacturer %x\n", manufacturer);

		uint16_t deviceId = read16FDC(FDC2212_DEVICE_ID_REGADDR);
		Serial.printf("i2c device %x\n", deviceId);
		if (!(deviceId == 0x3054 || deviceId == 0x3055)) {
			Serial.printf("Wrong i2c device %x\n", deviceId);
			//two valid device ids for FDC2214 0x3054 and 0x3055
			return false;
		}
		Serial.println("i2c device FDC2212 present.");

		Serial.println("FDC2212 loadsettings.");
		loadSettings();
		Serial.println("FDC2212 setgain.");
		setGain();
		Serial.println("Initialized.");
		initialized = true;
		return true;
	}
	return false;
}

/**************************************************************************/
/*!
 @brief  Initalizes FDC2214
 */
/**************************************************************************/
void FDC2212::loadSettings(void) {

	//reset device
	write16FDC(FDC2212_RESET_DEV, 0b1000000000000000);  //reset device

	//0b00  00 0001                      RESERVED
	//0b0   0 00 0001                    Normal current drive (auto scan is enabled)
	//0b0   0 0 00 0001                  INTB_DIS - Do NOT disable interrupt pin
	//0b0   0 0 0 00 0001                RESERVED
	//0b0   0 0 0 0 00 0001              Use internal oscilator
	//0b1   1 0 0 0 0 00 0001            RESERVED
	//0b0   0 1 0 0 0 0 00 0001          full current mode
	//0b1   1 0 1 0 0 0 0 00 0001        RESERVED
	//0b0   0 1 0 1 0 0 0 0 00 0001      device is active - no sleep
	//0b00 00 0 1 0 1 0 0 0 0 00 0001    Contineous reads on CH0 CONFIG.ACTIVE_CHAN

	uint16_t ACTIVE_CHAN = 0b0000000000000000; // Active Channel Selection
											   //    Selects channel for continuous conversions when
											   //    MUX_CONFIG.SEQUENTIAL is 0.
											   //    b00: Perform continuous conversions on Channel 0

	uint16_t SLEEP_MODE_EN = 0b0000000000000000; // Sleep mode enable b0: Device is active.
	uint16_t RESERVED_1 = 0b0001000000000000; // reserved set to 1
	uint16_t SENSOR_ACTIVATE_SEL = 0b0000000000000000; // Full Current Activation Mode b0: b0: Full Current Activation Mode – the FDC will drive maximum
													   //    sensor current for a shorter sensor activation time.

	uint16_t RESERVED_2 = 0b0000010000000000; // reserved set to 1
	uint16_t REF_CLK_SRC = 0b0000000000000000; // Select Reference Frequency Source
											   //    b0: Use Internal oscillator as reference frequency

	uint16_t RESERVED_3 = 0b0000000000000000; // reserved set to 0
	uint16_t INTB_DIS = 0b0000000000000000; // enable interrupt
	uint16_t HIGH_CURRENT_DRV = 0b0000000000000010; // High Current Sensor Drive
													//   b0: The FDC will drive all channels with normal sensor current
													//       (1.5mA max).
	uint16_t RESERVED_4 = 0b00000000000000001; // reserved set to 1

	uint16_t FDC2212_CONFIG_REGADDR_VAL = ACTIVE_CHAN | SLEEP_MODE_EN
			| RESERVED_1 | SENSOR_ACTIVATE_SEL | RESERVED_2 | REF_CLK_SRC
			| RESERVED_3 | INTB_DIS | HIGH_CURRENT_DRV | RESERVED_4;

	//FDC2212_CONFIG_REGADDR              0x1A
	write16FDC(FDC2212_CONFIG_REGADDR, FDC2212_CONFIG_REGADDR_VAL); //set config
	//write16FDC(FDC2212_CONFIG_REGADDR, 0x1E81);  //set config
//    write16FDC(FDC2214_CONFIG_REGADDR, 0x201);  //set config

	uint16_t readConf = read16FDC(FDC2212_CONFIG_REGADDR);  //set config
	Serial.printf("Conf register value: %d %x ", readConf, readConf);
	Serial.print(readConf, BIN);
	Serial.println();

	Serial.printf("Conf register value: %d %x ", FDC2212_CONFIG_REGADDR_VAL,
			FDC2212_CONFIG_REGADDR_VAL);
	Serial.print(FDC2212_CONFIG_REGADDR_VAL, BIN);
	Serial.println();

	//settle count maximized, slow application
	write16FDC(FDC2212_SETTLECOUNT_CH0_REGADDR, 0x64);

	//rcount maximized for highest accuracy
	write16FDC(FDC2212_RCOUNT_CH0_REGADDR, 0xFFFF);

	//no offset
	write16FDC(FDC2212_OFFSET_CH0_REGADDR, 0x0000);

	//set clock divider
	//0b00 0000 0001                                    Set clock div to 1
	//0b00 [00 0000 0001]                               RESERVED
	//0b01 [10] [00 0000 0001]                          divide by 2
	//0b01 0b100000000001
	//write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0x1001);
	write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0b100000000001);

	//set drive register
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0xF800);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0010100000000000);
	write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);

	// 0b 										101 					Deglitch 10MHz, oscilation is ~4MHz by default
	// 0b 		 	 	 0001000001 101           RESERVED
	// 0b00 		00 0001000001 101        		Ch0, Ch1 Sequence unused only read on CH0 using CONFIG.ACTIVE_CHAN
	// 0b0    0 00 0001000001 101        		Continuous conversion on the single channel selected by CONFIG.ACTIVE_CHAN register field.

	write16FDC(FDC2212_MUX_CONFIG_REGADDR, 0b0000001000001101); //set mux config for channels

	// set warnings
	// 0b1    											1										report data ready flag
	// 0b0000 								 0000 1										RESERVED
	// 0b1 									 1 0000 1										watchdog timeout error enable
	// 0b00000 				 00000 1 0000 1										RESERVED
	// 0b1 					 1 00000 1 0000 1										enable amplitude low warning
	// 0b1 				 1 1 00000 1 0000 1										enable amplitude high warning
	// 0b0 			 0 1 1 00000 1 0000 1										disable watchdog timeout error
	// 0b00 	00 0 1 1 00000 1 0000 1										RESERVED

	uint16_t ERROR_CONFIG_RES1 = 0b0;                  // reserved
													   //    b00
	uint16_t WD_ERR2OUT = 0b0 >> 2; // Watchdog Timeout Error to Output Register
									//    b0 : Do not report Watchdog Timeout errors in the DATA_CHx registers
	uint16_t AH_WARN2OUT = 0b0 >> 3; // Amplitude High Warning to Output Register
									 //    b0 : Do not report Amplitude High warnings in the DATA_CHx registers
	uint16_t AL_WARN2OUT = 0b0 >> 4; // Amplitude Low Warning to Output Register
									 //    b0 : Do not report Amplitude Low warnings in the DATA_CHx registers
	uint16_t ERROR_CONFIG_RES2 = 0b0 >> 5;          // reserved
													//    b00000
	uint16_t WD_ERR2INT = 0b0 >> 10;           // Watchdog Timeout Error to INTB
											   //    b0 : Do not report Under-range errors by asserting INTB pin and STATUS register.
	uint16_t ERROR_CONFIG_RES3 = 0b0 >> 11;             // reserved
														//    b0000
	uint16_t DRDY_2INT = 1 >> 15; // Data Ready Flag to INTB b0: Do not report Data Ready Flag by asserting INTB pin and STATUS register.
								  //    b1: Report Data Ready Flag by asserting INTB pin and updating STATUS. DRDY register field.
	uint16_t FDC2212_ERROR_CONFIG_VAL = ERROR_CONFIG_RES1 | WD_ERR2OUT
			| AH_WARN2OUT | AL_WARN2OUT | ERROR_CONFIG_RES2 | WD_ERR2INT
			| ERROR_CONFIG_RES3 | DRDY_2INT;

	//write16FDC(FDC2212_ERROR_CONFIG, 0b0001100000100001);  //set error config

	FDC2212_ERROR_CONFIG_VAL = 0b0001100000100001;
	write16FDC(FDC2212_ERROR_CONFIG, FDC2212_ERROR_CONFIG_VAL); //set error config

	uint16_t errorConfVal = read16FDC(FDC2212_ERROR_CONFIG);  //set config
	Serial.printf("Error Conf register value: %d %x ", FDC2212_ERROR_CONFIG_VAL,
			FDC2212_ERROR_CONFIG_VAL);
	Serial.print(FDC2212_ERROR_CONFIG_VAL, BIN);
	Serial.println();

	Serial.printf("Error Conf register value: %d %x ", errorConfVal,
			errorConfVal);
	Serial.print(errorConfVal, BIN);
	Serial.println();
}

///**************************************************************************/
///*!
//    @brief  Given a reading calculates the sensor frequency
//*/
///**************************************************************************/
//long long FDC2212::calculateFsensor(unsigned long reading){
////    Serial.println("reading: "+ String(reading));
//    //fsensor = (CH_FIN_SEL * fref * data) / 2 ^ 28
//    //should be mega hz so can truncate to long long
//    Serial.println("FDC reading: " + String(reading));
//    unsigned long long temp;
//    temp = 1 * 40000000 * reading;
//    temp = temp / (2^28);
////    Serial.println("frequency: " + String((long)temp));
//    return temp;
//}

///**************************************************************************/
///*!
//    @brief  Given sensor frequency calculates capacitance
//*/
///**************************************************************************/
//double FDC2212::calculateCapacitance(long long fsensor){
//    //differential configuration
//    //c sensor = 1                            - (Cboard + Cparacitic)
//    //             / (L * (2*pi * fsensor)^2)
//
//    double pi = 3.14159265359;
//    double L = 18; //uH
//    double Cboard = 33; //pf
//    double Cparacitic = 3; //pf
//
//    double temp = 2 * pi * fsensor;
//    temp = temp * temp;
//
//    temp = temp / 1000000; //uH
//    temp *= L;
//
////    Serial.println("capacitance: " + String(temp));
//    return temp;
//
//}

/**************************************************************************/
/*!
 @brief  Takes a reading and calculates capacitance from i
 */
/**************************************************************************/
IRAM_ATTR uint32_t FDC2212::getReading() {
	uint32_t tempReading = 0;
	if (!this->isReading) {
		readTimeMs = millis();
		this->isReading = true;
		int timeout = 100;

		//blink(1);
		int status = read16FDC(FDC2212_STATUS_REGADDR);

		if (status & FDC2212_CH0_DRDY)
			Serial.println("FDC2212_CH0_DRDY");
		if (status & FDC2212_CH0_AMPL_LOW)
			Serial.println("FDC2212_CH0_AMPL_LOW");
		if (status & FDC2212_CH0_AMPL_HIGH)
			Serial.println("FDC2212_CH0_AMPL_HIGH");
		if (status & FDC2212_CH0_WCHD_TO)
			Serial.println("FDC2212_CH0_WCHD_TO");

		while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
			status = read16FDC(FDC2212_STATUS_REGADDR);
			timeout--;
		}
		if (timeout == 100) {
			//could be stale grab another
			//read the 28 bit result
			tempReading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
			tempReading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);
			while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
//        Serial.println("status: " + String(status));
				status = read16FDC(FDC2212_STATUS_REGADDR);
				timeout--;
			}
		}
		if (timeout) {
			//read the 28 bit result
			tempReading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
			tempReading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);

			if (lastReading == 0) {
				lastReading = tempReading;
			}
		} else {
			//error not reading
			Serial.println("error reading fdc");
			tempReading = 0;
		}

		this->lastReadingTick = millis();
		this->readTimeMs = millis() - readTimeMs;

		if (readTimeMs < 20 && (tempReading < 40000000)) {
			reading = tempReading;
			this->dT = millis() - this->lastReadingTick;
			this->dCap_dT = ((float) reading - (float) this->lastReading) / dT;
			//Serial.printf("ms: %lu value: %u\n", readTimeMs,reading);
			if (reading < capMin)
				capMin = reading;
			if (reading > capMax)
				capMax = reading;

			// newavrg = eta * (newval) + (1-eta) * oldavrg
			if (reading != 0) {
				//reading = reading - 300000000;
				if (capSlow == 0)
					capSlow = (float) reading;
				else
					capSlow = slowFilter * (float) reading
							+ (1 - slowFilter) * capSlow;

				if (capFast == 0)
					capFast = (float) reading;
				else
					capFast = fastFilter * (float) reading
							+ (1 - fastFilter) * capFast;
			}
			/*
			 Serial.println();
			 Serial.print(capSlow - capFast);
			 Serial.print(" ");
			 Serial.print((_response.status == true)?"ON":"OFF");
			 */
			if (abs(capSlow - capFast) < triggerDiff
					&& _response.status == false) {
				_response.difference = abs(capSlow - capFast);
				_response.timeMs = millis();
				_response.status = true;
				_on_trigger(_response);
			}
			if (capSlow - capFast > triggerDiff && _response.status == true) {
				_response.difference = abs(capSlow - capFast);
				_response.timeMs = millis();
				_response.status = false;
				_on_trigger(_response);
			}

		}
	}
	this->shouldread = false;
	this->isReading = false;
	this->lastReading = reading;
	return reading;
}

///**************************************************************************/
///*!
//    @brief  Takes a reading and calculates capacitance from it
//*/
///**************************************************************************/
//double FDC2212::readCapacitance() {
//    int timeout = 100;
//    unsigned long reading = 0;
//    long long fsensor = 0;
//    int status = read16FDC(FDC2214_STATUS_REGADDR);
//    while (timeout && !(status & FDC2214_CH0_UNREADCONV)) {
////        Serial.println("status: " + String(status));
//        status = read16FDC(FDC2214_STATUS_REGADDR);
//        timeout--;
//    }
//    if (timeout) {
//        //read the 28 bit result
//        reading = read16FDC(FDC2214_DATA_CH0_REGADDR) << 16;
//        reading |= read16FDC(FDC2214_DATA_LSB_CH0_REGADDR);
//        fsensor = calculateFsensor(reading);
//        return calculateCapacitance(fsensor);
//    } else {
//        //error not reading
//        Serial.println("error reading fdc");
//        return 0;
//    }
//}

/**************************************************************************/
/*!
 @brief  Scans various gain settings until the amplitude flag is cleared.
 WARNING: Changing the gain setting will generally have an impact on the
 reading.
 */
/**************************************************************************/
void FDC2212::setGain(void) {
	//todo
}
/**************************************************************************/
/*!
 @brief  I2C low level interfacing
 */
/**************************************************************************/

// Read 2 byte from the VL6180X at 'address'
uint16_t FDC2212::read16FDC(uint8_t address) {
	/*
	 uint16_t Address;
	 Address = FDC2214_I2C_ADDRESS << 1;
	 Address |= OAR1_ADD0_Set;
	 uint16_t data;
	 Wire.beginTransmission(Address);
	 Wire.write(address);
	 Wire.endTransmission();

	 Wire.requestFrom(Address, (uint8_t) 2);
	 while (!Wire.available());
	 data = Wire.read();
	 data <<= 8;
	 while (!Wire.available());
	 data |= Wire.read();
	 return data;
	 */

	/*
	 uint8_t buffer[2];
	 ma_read_uint16t(address,buffer);
	 */
	uint8_t buffer[2];
	myI2C.readBytes(FDC2214_I2C_ADDRESS, address, 2, buffer);
	uint16_t result = (buffer[0] << 8 | buffer[1]);
	return result;
}

// write 2 bytes
void FDC2212::write16FDC(uint8_t address, uint16_t data) {
	/*
	 uint16_t Address = FDC2214_I2C_ADDRESS << 1;
	 Address &= OAR1_ADD0_Reset;

	 Wire.beginTransmission(Address);
	 Wire.write(address >> 8);
	 Wire.write(address & 0xFF);
	 Wire.write(data >> 8);
	 Wire.write(data);
	 Wire.endTransmission();
	 */
	//ma_write_uint16t(address, data);
	uint8_t buffer[] = { (uint8_t) (data >> 8), (uint8_t) data };
	myI2C.writeBytes(FDC2214_I2C_ADDRESS, address, 2, buffer);
}

void FDC2212::shouldRead() {
	this->shouldread = true;
}
