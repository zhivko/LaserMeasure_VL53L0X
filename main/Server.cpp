//  python /home/klemen/esp/arduino-esp32/tools/espota.py -I 192.168.43.21 -i 192.168.43.96 -p 3232 -P 3232 -f /home/klemen/esp/hello_world/build/hello-world.bin -d
// platformio run --target uploadfs
// C:\Users\klemen\Dropbox\Voga\BleVogaLifter-esp32-DRV8703Q>c:\Python27\python.exe c:\Users\klemen\.platformio\packages\framework-arduinoespressif32\tools\esptool.py --chip esp32 --port COM3 --baud 115200 --before default_reset --after hard_reset erase_flash
// https://github.com/thehookup/ESP32_Ceiling_Light/blob/master/GPIO_Limitations_ESP32_NodeMCU.jpg
// Need to test: VL53L0X
// https://esp32.com/viewtopic.php?f=13&t=2525#p12056

// cd ~/esp/openocd-esp32
// bin/openocd -s share/openocd/scripts -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f board/esp-wroom-32.cfg -d3

#include <WiFi.h>
#include <FS.h>
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <stdint.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/timers.h"

#include "TaskCore0.h"
//#include "I2CTask.h"

#include <Preferences.h>
#include "nvs_flash.h"

#include "FDC2212.h"

String ssid;
String password;
Preferences preferences;
bool reportingJson = false;

const char softAP_ssid[] = "MLIFT";
const char softAP_password[] = "Doitman1";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
const char * mysystem_event_names[] = { "WIFI_READY", "SCAN_DONE", "STA_START",
		"STA_STOP", "STA_CONNECTED", "STA_DISCONNECTED", "STA_AUTHMODE_CHANGE",
		"STA_GOT_IP", "STA_LOST_IP", "STA_WPS_ER_SUCCESS", "STA_WPS_ER_FAILED",
		"STA_WPS_ER_TIMEOUT", "STA_WPS_ER_PIN", "AP_START", "AP_STOP",
		"AP_STACONNECTED", "AP_STADISCONNECTED", "AP_PROBEREQRECVED", "GOT_IP6",
		"ETH_START", "ETH_STOP", "ETH_CONNECTED", "ETH_DISCONNECTED",
		"ETH_GOT_IP", "MAX" };

int NO_AP_FOUND_count = 0;

static const int spiClk = 1000000; // 1 MHz
uint16_t toTransfer;
//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

int shouldStopM1 = 0;
int shouldStopM2 = 0;
int shouldPwm_M1_left = 0;
int shouldPwm_M1_right = 0;
int shouldPwm_M2_left = 0;
int shouldPwm_M2_right = 0;

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_10_BIT  10
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define PWM1_PIN GPIO_NUM_12
#define PWM2_PIN GPIO_NUM_14
#define PWM3_PIN GPIO_NUM_26
#define PWM4_PIN GPIO_NUM_27
#define LED_PIN GPIO_NUM_2
#define GATEDRIVER_PIN GPIO_NUM_32

#define LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define pwmDelta     5

#define SS1 33
#define SS2 25

#define ROTARY_ENCODER2_A_PIN GPIO_NUM_4
#define ROTARY_ENCODER2_B_PIN GPIO_NUM_16
#define ROTARY_ENCODER1_A_PIN GPIO_NUM_17
#define ROTARY_ENCODER1_B_PIN GPIO_NUM_5

// @doc https://remotemonitoringsystems.ca/time-zone-abbreviations.php
// @doc timezone UTC = UTC
const char* NTP_SERVER0 = "0.si.pool.ntp.org";
const char* NTP_SERVER1 = "1.si.pool.ntp.org";
const char* NTP_SERVER2 = "2.si.pool.ntp.org";
const char* TZ_INFO2 = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";
time_t now;
struct tm info;

volatile uint16_t pwmValueMax = 1024;

volatile int16_t pwm1 = 0;       // how bright the LED is
volatile int16_t pwm2 = 0;       // how bright the LED is
volatile int fadeAmount = 1;     // how many points to fade the LED by

String status1;
String status2;
String gdfVds1;
String gdfVds2;

volatile int16_t encoder1_value;
volatile int16_t encoder2_value;
//Ticker mover;
//Ticker jsonReporter;
TaskHandle_t TaskA;
TaskHandle_t reportJsonTask;
//TaskHandle_t i2cTask;

volatile double output1, output2;
volatile double target1, target2;
volatile bool pidEnabled = true;

AiEsp32RotaryEncoder rotaryEncoder2 = AiEsp32RotaryEncoder(
ROTARY_ENCODER2_A_PIN, ROTARY_ENCODER2_B_PIN, -1, -1);
AiEsp32RotaryEncoder rotaryEncoder1 = AiEsp32RotaryEncoder(
ROTARY_ENCODER1_A_PIN, ROTARY_ENCODER1_B_PIN, -1, -1);

MiniPID pid1 = MiniPID(0.0, 0.0, 0.0);
MiniPID pid2 = MiniPID(0.0, 0.0, 0.0);

String txtToSend;

uint16_t an1, an2;
float an1_fast, an1_slow;
float an2_fast, an2_slow;
uint16_t an1_max, an2_max;
uint16_t stop2_top, stop2_bottom;
uint16_t stop1_top, stop1_bottom;
String status;
String previousPercent_str_1;
String previousPercent_str_2;
long searchTopMilis;

int16_t deltaSearch = 2000;

FDC2212 fdc2212;

uint32_t cap_reading = 0;

//AsyncPing myPing;
//IPAddress addr;

#define TAG "TIME"
int id = 1;
TimerHandle_t tmr;

int interval = 5000;
void timerCallBack(TimerHandle_t xTimer) {
	ESP_LOGI(TAG,"tring tring!!!");
}

IRAM_ATTR void reportJson(void *pvParameters) {
	for (;;) {
		//Serial.println("reportjson1");
		if (ws.count() > 0) {
			//Serial.println("reportjson");
			//reportingJson = true;reportJson
			txtToSend = "";
			txtToSend.concat("{");

			txtToSend.concat("\"encoder1_value\":");
			txtToSend.concat(encoder1_value);
			txtToSend.concat(",");

			txtToSend.concat("\"encoder2_value\":");
			txtToSend.concat(encoder2_value);
			txtToSend.concat(",");

			txtToSend.concat("\"pwm1\":");
			txtToSend.concat(pwm1);
			txtToSend.concat(",");

			txtToSend.concat("\"pwm2\":");
			txtToSend.concat(pwm2);
			txtToSend.concat(",");

			txtToSend.concat("\"target1\":");
			txtToSend.concat(target1);
			txtToSend.concat(",");

			txtToSend.concat("\"target2\":");
			txtToSend.concat(target2);
			txtToSend.concat(",");

			txtToSend.concat("\"output1\":");
			txtToSend.concat(output1);
			txtToSend.concat(",");

			txtToSend.concat("\"output2\":");
			txtToSend.concat(output2);
			txtToSend.concat(",");

			txtToSend.concat("\"an1\":");
			txtToSend.concat(an1_slow);
			txtToSend.concat(",");
			txtToSend.concat("\"an2\":");
			txtToSend.concat(an2_slow);
			txtToSend.concat(",");

			txtToSend.concat("\"actual_diff\":");
			txtToSend.concat(pid1.getActual() - pid2.getActual());
			txtToSend.concat(",");

			txtToSend.concat("\"PID1output\":");
			txtToSend.concat("\"Pout=");
			txtToSend.concat(pid1.getPoutput());
			txtToSend.concat("<br>Iout=");
			txtToSend.concat(pid1.getIoutput());
			txtToSend.concat("<br>Dout=");
			txtToSend.concat(pid1.getDoutput());
			txtToSend.concat("<br>Fout=");
			txtToSend.concat(pid1.getFoutput());
			txtToSend.concat("<br>POSout=");
			txtToSend.concat(pid1.getPOSoutput());
			txtToSend.concat("<br>POSoutF=");
			txtToSend.concat(pid1.getPOSoutputFiltered());
			txtToSend.concat("<br>setpoint=");
			txtToSend.concat(pid1.getSetpoint());
			txtToSend.concat("<br>actual=");
			txtToSend.concat(pid1.getActual());
			txtToSend.concat("<br>error=");
			txtToSend.concat(pid1.getError());
			txtToSend.concat("<br>errorSum=");
			txtToSend.concat(pid1.getErrorSum());
			txtToSend.concat("<br>maxIOutput=");
			txtToSend.concat(pid1.getMaxIOutput());
			txtToSend.concat("\",");

			txtToSend.concat("\"PID2output\":");
			txtToSend.concat("\"Pout=");
			txtToSend.concat(pid2.getPoutput());
			txtToSend.concat("<br>Iout=");
			txtToSend.concat(pid2.getIoutput());
			txtToSend.concat("<br>Dout=");
			txtToSend.concat(pid2.getDoutput());
			txtToSend.concat("<br>Fout=");
			txtToSend.concat(pid2.getFoutput());
			txtToSend.concat("<br>POSout=");
			txtToSend.concat(pid2.getPOSoutput());
			txtToSend.concat("<br>POSoutF=");
			txtToSend.concat(pid2.getPOSoutputFiltered());
			txtToSend.concat("<br>setpoint=");
			txtToSend.concat(pid2.getSetpoint());
			txtToSend.concat("<br>actual=");
			txtToSend.concat(pid2.getActual());
			txtToSend.concat("<br>error=");
			txtToSend.concat(pid2.getError());
			txtToSend.concat("<br>errorSum=");
			txtToSend.concat(pid2.getErrorSum());
			txtToSend.concat("<br>maxIOutput=");
			txtToSend.concat(pid2.getMaxIOutput());
			txtToSend.concat("\",");

			txtToSend.concat("\"stop1_top\":");
			txtToSend.concat(stop1_top);
			txtToSend.concat(",");
			txtToSend.concat("\"stop1_bottom\":");
			txtToSend.concat(stop1_bottom);
			txtToSend.concat(",");
			txtToSend.concat("\"stop2_top\":");
			txtToSend.concat(stop2_top);
			txtToSend.concat(",");
			txtToSend.concat("\"stop2_bottom\":");
			txtToSend.concat(stop2_bottom);
			txtToSend.concat(",");

			txtToSend.concat("\"cap_reading\":");
			txtToSend.concat(fdc2212.reading);
			txtToSend.concat(",");
			txtToSend.concat("\"cap_read_time_ms\":");
			txtToSend.concat(fdc2212.readTimeMs);
			txtToSend.concat(",");

			txtToSend.concat("\"capfast\":");
			txtToSend.concat(fdc2212.capFast);
			txtToSend.concat(",");
			txtToSend.concat("\"capslow\":");
			txtToSend.concat(fdc2212.capSlow);
			txtToSend.concat(",");

			txtToSend.concat("\"esp32_heap\":");
			txtToSend.concat(ESP.getFreeHeap());
			txtToSend.concat("}");

			/*
			 size_t len = txtToSend.len;
			 AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len); //  creates a buffer (len + 1) for you.
			 if (buffer) {
			 root.printTo((char *)buffer->get(), len + 1);
			 if (client) {
			 client->text(buffer);
			 } else {
			 ws.textAll(buffer);
			 }
			 };
			 *
			 */
			ws.textAll(txtToSend.c_str());
			yield();
			//reportingJson = false;
		}
		//Serial.println("reportjson2");
		unsigned long start;
		long delta;

		//long randNumber = random(0, 10);
		//Serial.println("reportjson3");
		if (rotaryEncoder1.encoderChanged() != 0
				&& ((int) target1) == encoder1_value) {
			//Serial.println("Saving to flash enc1.");
			encoder1_value = rotaryEncoder1.readEncoder();
			start = micros(); // ref: https://github.com/espressif/arduino-esp32/issues/384
			preferences.begin("settings", false);
			preferences.putInt("encoder1_value", rotaryEncoder1.readEncoder());
			preferences.putInt("target1", (int) target1);
			preferences.end();
			delta = micros() - start;

			if (delta > 1000)
				Serial.printf("%lu Preferences save completed in %lu us.\n",
						micros(), delta);

			yield();
		}
		if (rotaryEncoder2.encoderChanged() != 0
				&& ((int) target2) == encoder2_value) {
			//Serial.println("Saving to flash enc2.");
			encoder2_value = rotaryEncoder2.readEncoder();
			start = micros(); // ref: https://github.com/espressif/arduino-esp32/issues/384
			preferences.begin("settings", false);
			preferences.putInt("encoder2_value", rotaryEncoder2.readEncoder());
			preferences.putInt("target2", (int) target2);
			preferences.end();
			delta = micros() - start;

			if (delta > 1000)
				Serial.printf("%lu Preferences save completed in %lu us.\n",
						micros(), delta);

			yield();
		}
		esp_task_wdt_reset();
		vTaskDelay(400 / portTICK_PERIOD_MS);
	}
}

String getToken(String data, char separator, int index) {
	int found = 0;
	int strIndex[] = { 0, -1 };
	int maxIndex = data.length() - 1;

	for (int i = 0; i <= maxIndex && found <= index; i++) {
		if (data.charAt(i) == separator || i == maxIndex) {
			found++;
			strIndex[0] = strIndex[1] + 1;
			strIndex[1] = (i == maxIndex) ? i + 1 : i;
		}
	}

	String ret("");
	if (found > index) {
		ret = data.substring(strIndex[0], strIndex[1]);
	}
	return ret;
}

void setPidsFromString(String input) {
	Serial.printf("Parsing pid: %s\n", input.c_str());
	String p_str = getToken(input, ' ', 0);
	String p_val = getToken(p_str, '=', 1);

	String i_str = getToken(input, ' ', 1);
	String i_val = getToken(i_str, '=', 1);

	String d_str = getToken(input, ' ', 2);
	String d_val = getToken(d_str, '=', 1);

	String f_str = getToken(input, ' ', 3);
	String f_val = getToken(f_str, '=', 1);

	String syn_str = getToken(input, ' ', 4);
	String syn_val = getToken(syn_str, '=', 1);

	String synerr_str = getToken(input, ' ', 5);
	String synerr_val = getToken(synerr_str, '=', 1);

	String ramp_str = getToken(input, ' ', 6);
	String ramp_val = getToken(ramp_str, '=', 1);
	if (ramp_val.equals("")) {
		ramp_val = "1.0";
	}

	pid1.setPID(p_val.toFloat(), i_val.toFloat(), d_val.toFloat(),
			f_val.toFloat());
	pid2.setPID(p_val.toFloat(), i_val.toFloat(), d_val.toFloat(),
			f_val.toFloat());
	pid1.setOutputRampRate(ramp_val.toFloat());
	pid2.setOutputRampRate(ramp_val.toFloat());
	//pid1.setOutputFilter(0.01);
	//pid2.setOutputFilter(0.01);

	Serial.print("SynError: ");
	Serial.println(synerr_val.toFloat());

	pid1.setSyncDisabledForErrorSmallerThen(synerr_val.toFloat());
	pid2.setSyncDisabledForErrorSmallerThen(synerr_val.toFloat());

	if (syn_val.equals("1")) {
		pid1.setSynchronize(true);
		pid2.setSynchronize(true);
	} else {
		pid1.setSynchronize(false);
		pid2.setSynchronize(false);
	}
}

void sendPidToClient() {
	String txtToSend = "";
	txtToSend.concat("{");
	txtToSend.concat("\"pid\":");
	txtToSend.concat("\"p=");
	txtToSend.concat(pid1.getP());
	txtToSend.concat(" i=");
	txtToSend.concat(pid1.getI());
	txtToSend.concat(" d=");
	txtToSend.concat(pid1.getD());
	txtToSend.concat(" f=");
	txtToSend.concat(pid1.getF());
	txtToSend.concat(" syn=");
	txtToSend.concat(pid1.getSynchronize() ? "1" : "0");
	txtToSend.concat(" synErr=");
	txtToSend.concat(pid1.getSyncDisabledForErrorSmallerThen());
	txtToSend.concat(" ramp=");
	txtToSend.concat(pid1.getRampRate());
	txtToSend.concat("\",");
	txtToSend.concat("\"maxPercentOutput\":");
	txtToSend.concat((int) (ceil(pid1.getMaxOutput() / pwmValueMax * 100.0)));
	txtToSend.concat("}");
	ws.textAll(txtToSend.c_str());

}

void setOutputPercent(String percent_str, int i) {
	int outputMin = -(int) (pwmValueMax * percent_str.toFloat() / 100.0);
	int outputMax = (int) (pwmValueMax * percent_str.toFloat() / 100.0);
	Serial.printf("outputMin=%d outputMax=%d\n", outputMin, outputMax);
	if (i == 1) {
		pid1.setOutputLimits(outputMin, outputMax);
	} else {
		pid2.setOutputLimits(outputMin, outputMax);
	}
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
	Serial.printf("Listing directory: %s\r\n", dirname);

	File root = fs.open(dirname);
	if (!root) {
		Serial.println("- failed to open directory");
		return;
	}
	if (!root.isDirectory()) {
		Serial.println(" - not a directory");
		return;
	}

	File file = root.openNextFile();
	while (file) {
		if (file.isDirectory()) {
			Serial.print("  DIR : ");
			Serial.println(file.name());
			if (levels) {
				listDir(fs, file.name(), levels - 1);
			}
		} else {
			Serial.print("  FILE: ");
			Serial.print(file.name());
			Serial.print("\tSIZE: ");
			Serial.println(file.size());
		}
		file = root.openNextFile();
	}
}

void clearFault() {
	digitalWrite(SS1, LOW);
	// SPI WRITE
	vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

	byte data_read = B00000000;  // WRITE OPERATION
	byte data_address = B00010000; // ADDRES 02x MAIN REGISTER
	byte data = data_read | data_address;

	uint16_t data_int = data << 8 | B00000001;  // B00000001 ... CLR_FLT
												// B00000010 ... IN2/EN
												// B00000100 ... IN1/PH
												// B00111000 ... LOCK

	vspi->transfer16(data_int);
	vspi->endTransaction();
	digitalWrite(SS1, HIGH);
}

String getfault(uint16_t reply) {
	String status1 = "";
	// fault bytes:
	uint8_t FAULT_FAULT = B1 << 7; // FAULT R 0b Logic OR of the FAULT status register excluding the OTW bit
	uint8_t FAULT_WDFLT = B1 << 6;     // WDFLT R 0b Watchdog time-out fault
	uint8_t FAULT_GDF = B1 << 5; // GDF R 0b Indicates gate drive fault condition
	uint8_t FAULT_OCP = B1 << 4; // OCP R 0b Indicates VDS monitor overcurrent fault condition
	uint8_t FAULT_VM_UVFL = B1 << 3; // VM_UVFL R 0b Indicates VM undervoltage lockout fault condition
	uint8_t FAULT_VCP_UVFL = B1 << 2; // VCP_UVFL R 0b Indicates charge-pump undervoltage fault condition
	uint8_t FAULT_OTSD = B1 << 1; // OTSD R 0b Indicates overtemperature shutdown
	uint8_t FAULT_OTW = B1 << 0;   // OTW R 0b Indicates overtemperature warning
	if ((reply & FAULT_FAULT) > 0) {
		status1.concat(
				"Logic OR of the FAULT status register excluding the OTW bit\n");
	}
	if ((reply & FAULT_WDFLT) > 0) {
		status1.concat("Watchdog time-out fault\n");
	}
	if ((reply & FAULT_GDF) > 0) {
		status1.concat("Gate drive fault\n");
	}
	if ((reply & FAULT_OCP) > 0) {
		status1.concat("VDS monitor overcurrent fault\n");
	}
	if ((reply & FAULT_VM_UVFL) > 0) {
		status1.concat("VM undervoltage lockout fault\n");
	}
	if ((reply & FAULT_VCP_UVFL) > 0) {
		status1.concat("Charge-pump undervoltage fault\n");
	}
	if ((reply & FAULT_OTSD) > 0) {
		status1.concat("Overtemperature shutdown\n");
	}
	if ((reply & FAULT_OTW) > 0) {
		status1.concat("Overtemperature warning\n ");
	}
	return status1;
}

void testSpi(int which) {
	usleep(1);
	if (which == 1)
		digitalWrite(SS1, LOW);
	else
		digitalWrite(SS2, LOW);
	// SPI WRITE
	vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

	// http://www.ti.com/product/drv8702-q1?qgpn=drv8702-q1
	// datasheet: http://www.ti.com/lit/gpn/drv8702-q1
	// page 42
	byte data_read = B10000000;  // READ OPERATION
	byte data_address = B00010000; // ADDRES 02x MAIN REGISTER
	byte data = data_read | data_address;
	byte lowbyte = B0;
	uint16_t data_int = data << 8 | lowbyte;

	uint16_t reply = vspi->transfer16(data_int); // should return 0x18 B00011000
	vspi->endTransaction();

	usleep(1);
	if (reply == B00011000) {
		Serial.println("YES it is ON!");
		if (which == 1)
			status1.concat("DRV8703Q is ON (Not locked)\n");
		else
			status2.concat("DRV8703Q is ON (Not locked)\n");

		Serial.println("----");
		Serial.println(status1);
		Serial.println(status2);
		Serial.println("----");

		usleep(1);

		// SPI WRITE
		vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

		data_read = B10000000;  // READ OPERATION
		data_address = B00000000; // ADDRES 0x FAULT REGISTER
		data = data_read | data_address;
		lowbyte = B0;
		data_int = data << 8 | lowbyte;

		reply = vspi->transfer16(data_int);  // should return 0x18
		vspi->endTransaction();
		if (which == 1)
			digitalWrite(SS1, HIGH);
		else
			digitalWrite(SS2, HIGH);
		usleep(1);
		Serial.print("SPI reply: ");
		Serial.println(reply, BIN);

		if (which == 1)
			status1.concat(getfault(reply));
		else
			status2.concat(getfault(reply));

		Serial.println("----");
		Serial.println(status1);
		Serial.println(status2);
		Serial.println("----");
	} else {
		if (which == 1)
			status1.concat("DRV8703Q is NOT ON\n");
		else
			status2.concat("DRV8703Q is NOT ON\n");
	}

}

void gdfVdsStatus(int which) {
	usleep(1);
	if (which == 1)
		digitalWrite(SS1, LOW);
	else
		digitalWrite(SS2, LOW);
	// SPI WRITE
	vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

	// http://www.ti.com/product/drv8702-q1?qgpn=drv8702-q1
	// datasheet: http://www.ti.com/lit/gpn/drv8702-q1
	// page 42

	byte data_read = B10000000;  // READ OPERATION
	byte data_address = B00001000; // ADDRES 01x VDS and GDF Status Register Name
	byte data = data_read | data_address;
	byte lowbyte = B0;
	uint16_t data_int = data << 8 | lowbyte;

	uint16_t reply = vspi->transfer16(data_int); // should return GDF and VDS statuses
	vspi->endTransaction();
	if (which == 1)
		digitalWrite(SS1, HIGH);
	else
		digitalWrite(SS2, HIGH);
	usleep(1);
	String ret = "";

	// fault bytes:
	uint8_t H2_GDF = B1 << 7; // Gate drive fault on the high-side FET of half bridge 2
	uint8_t L2_GDF = B1 << 6; // Gate drive fault on the low-side FET of half bridge 2
	uint8_t H1_GDF = B1 << 5; // Gate drive fault on the high-side FET of half bridge 1
	uint8_t L1_GDF = B1 << 4; // Gate drive fault on the low-side FET of half bridge 1
	uint8_t H2_VDS = B1 << 3; // VDS monitor overcurrent fault on the high-side FET of half bridge 2
	uint8_t L2_VDS = B1 << 2; // VDS monitor overcurrent fault on the low-side FET of half bridge 2
	uint8_t H1_VDS = B1 << 1; // VDS monitor overcurrent fault on the high-side FET of half bridge 1
	uint8_t L1_VDS = B1 << 0; // VDS monitor overcurrent fault on the low-side FET of half bridge 1

	if (which == 1)
		digitalWrite(SS1, HIGH);
	else
		digitalWrite(SS2, HIGH);

	usleep(1);
	Serial.print("SPI reply: ");
	Serial.println(reply, BIN);

	if ((reply & H2_GDF) > 0) {
		ret.concat(" Gate drive fault on the high-side FET of half bridge 2\n");
	}
	if ((reply & L2_GDF) > 0) {
		ret.concat("Gate drive fault on the low-side FET of half bridge 2\n");
	}
	if ((reply & H1_GDF) > 0) {
		ret.concat("Gate drive fault on the high-side FET of half bridge 1\n");
	}
	if ((reply & L1_GDF) > 0) {
		ret.concat("Gate drive fault on the low-side FET of half bridge 1\n");
	}
	if ((reply & H2_VDS) > 0) {
		ret.concat(
				"VDS monitor overcurrent fault on the high-side FET of half bridge 2\n");
	}
	if ((reply & L2_VDS) > 0) {
		ret.concat(
				"VDS monitor overcurrent fault on the low-side FET of half bridge 2\n");
	}
	if ((reply & H1_VDS) > 0) {
		ret.concat(
				"VDS monitor overcurrent fault on the high-side FET of half bridge 1\n ");
	}
	if ((reply & L1_VDS) > 0) {
		ret.concat(
				"VDS monitor overcurrent fault on the low-side FET of half bridge 1\n ");
	}

	if (which == 1)
		gdfVds1 = ret;
	else
		gdfVds2 = ret;

}

String scanNetworks() {
	String ret;
	int n = WiFi.scanNetworks(false, true, false, 1500);

	Serial.println("scan done");
	if (n == 0) {
		ret.concat("no networks found");
	} else {
		for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
			ret.concat("wifi ");
			ret.concat(WiFi.SSID(i));
			ret.concat(" (");
			ret.concat(WiFi.RSSI(i));
			ret.concat(") ");
			ret.concat((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "PASS");
			ret.concat("\n");
		}
	}
	return ret;
}

void processWsData(char *data, AsyncWebSocketClient* client) {
	String input;
	input.concat(data);
	printf("received: %s\n", input.c_str());
	if (input.equals(String("status"))) {
		status1 = " ";
		status2 = " ";
		testSpi(1);
		Serial.println("Statuses:");
		Serial.println(status1);
		testSpi(2);
		Serial.println(status2);

		client->printf("motor1_pos %i", encoder1_value);
		client->printf("motor2_pos %i", encoder2_value);

		client->printf("status1: %s", status1.c_str());
		client->printf("status2: %s", status2.c_str());

		client->printf("shouldPwm_M1_left: %d", shouldPwm_M1_left);
		client->printf("shouldPwm_M1_right: %d", shouldPwm_M1_right);
		client->printf("shouldstop_M1: %d", shouldStopM1);
		client->printf("shouldPwm_M2_left: %d", shouldPwm_M2_left);
		client->printf("shouldPwm_M2_right: %d", shouldPwm_M2_right);
		client->printf("shouldstop_M2: %d", shouldStopM2);

	} else if (input.startsWith("gdfvdsstatus")) {
		gdfVdsStatus(1);
		gdfVdsStatus(2);
		client->printf("gdfvdsstatus1: %s", gdfVds1.c_str());
		client->printf("gdfvdsstatus2: %s", gdfVds2.c_str());
	} else if (input.startsWith("clrflt")) {
		clearFault();
		client->printf("Clear Fault done.");
	} else if (input.startsWith("pid#")) {
		String input2 = getToken(input, '#', 1);
		setPidsFromString(input2);
		sendPidToClient();

		preferences.begin("settings", false);
		preferences.putString("pid", input);
		preferences.end();

		//ws.textAll(JSONmessageBuffer);

		Serial.printf("Parsing pid done.");
	} else if (input.startsWith("gCodeCmd")) {
		Serial.printf("Parsing target1=.. target2=... command:%s\n",
				input.c_str());
		String input2 = getToken(input, '#', 1);
		String target1_str = getToken(input2, ' ', 0);
		String target1_duty = getToken(target1_str, '=', 1);
		String target2_str = getToken(input2, ' ', 1);
		String target2_duty = getToken(target2_str, '=', 1);

		target1 = (double) target1_duty.toFloat();
		target2 = (double) target2_duty.toFloat();
		Serial.print("target1= ");
		Serial.print(target1);
		Serial.print(" target2= ");
		Serial.println(target2);

	} else if (input.startsWith("enablePid")) {
		pidEnabled = true;
	} else if (input.startsWith("maxPercentOutput1")) {
		String percent_str = getToken(input, '#', 1);
		setOutputPercent(percent_str, 1);
		preferences.begin("settings", false);
		preferences.putInt("outputMin1", pid1.getMinOutput());
		preferences.putInt("outputMax1", pid1.getMaxOutput());
		preferences.end();
	} else if (input.startsWith("maxPercentOutput2")) {
		String percent_str = getToken(input, '#', 1);
		setOutputPercent(percent_str, 2);
		preferences.begin("settings", false);
		preferences.putInt("outputMin2", pid2.getMinOutput());
		preferences.putInt("outputMax2", pid2.getMaxOutput());
		preferences.end();
	} else if (input.startsWith("disablePid")) {
		pidEnabled = false;
	} else if (input.startsWith("wificonnect")) {
		ssid = getToken(input, ' ', 1);
		password = getToken(input, ' ', 2);

		preferences.begin("settings", false);
		preferences.putString("wifi_ssid", ssid);
		preferences.putString("wifi_password", password);
		preferences.end();

		printf("wificonnect ssid: %s, password: %s\n", ssid.c_str(),
				password.c_str());
		ws.textAll("MLIFT restart.");
		ws.closeAll();

		Serial.printf("wificonnect ssid: %s\n", ssid.c_str());
		delay(500);

		esp_restart();
	} else if (input.startsWith("pwm_m1_left")) {
		shouldStopM1 = 0;
		shouldPwm_M1_left = 1;
		shouldPwm_M1_right = 0;
	} else if (input.startsWith("pwm_m1_right")) {
		shouldStopM1 = 0;
		shouldPwm_M1_left = 0;
		shouldPwm_M1_right = 1;
	} else if (input.startsWith("stop pwm_m1")) {
		shouldStopM1 = 1;
		shouldPwm_M1_left = 0;
		shouldPwm_M1_right = 0;
	} else if (input.startsWith("pwm_m2_left")) {
		shouldStopM2 = 0;
		shouldPwm_M2_left = 1;
		shouldPwm_M2_right = 0;
	} else if (input.startsWith("pwm_m2_right")) {
		shouldStopM2 = 0;
		shouldPwm_M2_left = 0;
		shouldPwm_M2_right = 1;
	} else if (input.startsWith("stop pwm_m2")) {
		shouldStopM2 = 1;
		shouldPwm_M2_left = 0;
		shouldPwm_M2_right = 0;
	} else if (input.startsWith("gotop")) {
		target1 = stop1_top;
		target2 = stop1_top;
		pidEnabled = true;
	} else if (input.startsWith("gobottom")) {
		target1 = stop1_bottom;
		target2 = stop1_bottom;
		pidEnabled = true;
	} else if (input.startsWith("searchtop")
			|| input.startsWith("searchbottom")) {
		if (getToken(input, ' ', 1).equals(String("start"))) {
			pidEnabled = false;
			pwm1 = 0;
			pwm2 = 0;

			previousPercent_str_1 = String(
					(int) (ceil(pid1.getMaxOutput() / pwmValueMax * 100.0)));
			previousPercent_str_2 = String(
					(int) (ceil(pid2.getMaxOutput() / pwmValueMax * 100.0)));
			String percentPower = "70";
			status = getToken(input, ' ', 0);
			setOutputPercent(percentPower, 1);
			setOutputPercent(percentPower, 2);

			if (input.startsWith("searchtop")) {
				target1 = encoder1_value + deltaSearch;
				target2 = target1;
			} else {
				target1 = encoder1_value - deltaSearch;
				target2 = target1;
			}
			target2 = target1;
			pidEnabled = true;
			searchTopMilis = millis();
		} else {
			status = "";
			setOutputPercent(previousPercent_str_1, 1);
			setOutputPercent(previousPercent_str_1, 2);
		}
	} else if (input.startsWith("scan")) {
		printf("scan...\n");
		//vTaskSuspend(reportJsonTask);
		String scannedNetworks = scanNetworks();
		ws.textAll(scannedNetworks);
		//vTaskResume(reportJsonTask);
	}
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
		AwsEventType type, void * arg, uint8_t *data, size_t len) {
	if (type == WS_EVT_CONNECT) {
		//client connected
		printf("%lu ws[%s][%u] connect\n", millis(), server->url(),
				client->id());
		client->printf("Hello Client %u :)", client->id());
		client->ping();
		sendPidToClient();
	} else if (type == WS_EVT_DISCONNECT) {
		//client disconnected
		printf("%lu ws[%s][%u] disconnect\n", millis(), server->url(),
				client->id());
	} else if (type == WS_EVT_ERROR) {
		//error was received from the other end
		printf("%lu ws[%s][%u] error(%u): %s\n", millis(), server->url(),
				client->id(), *((uint16_t*) arg), (char*) data);
	} else if (type == WS_EVT_PONG) {
		//pong message was received (in response to a ping request maybe)
		printf("%lu ws[%s][%u] pong[%u]: %s\n", millis(), server->url(),
				client->id(), len, (len) ? (char*) data : "");
	} else if (type == WS_EVT_DATA) {
		//data packet
		AwsFrameInfo * info = (AwsFrameInfo*) arg;
		if (info->final && info->index == 0 && info->len == len) {
			//the whole message is in a single frame and we got all of it's data
			printf("%lu ws[%s][%u] %s-message[%llu]: ", millis(), server->url(),
					client->id(), (info->opcode == WS_TEXT) ? "text" : "binary",
					info->len);
			if (info->opcode == WS_TEXT) {
				data[len] = 0;
				printf("%s\n", (char*) data);
				processWsData((char*) data, client);
			} else {
				printf("not text\n");
				for (size_t i = 0; i < info->len; i++) {
					printf("%02x ", data[i]);
				}
				printf("\n");
			}
			/*
			 if(info->opcode == WS_TEXT)
			 client->text("I got your text message");
			 else
			 client->binary("I got your binary message");
			 */
		} else {
			printf("multi frames\n");
			//message is comprised of multiple frames or the frame is split into multiple packets
			if (info->index == 0) {
				if (info->num == 0)
					printf("%lu ws[%s][%u] %s-message start\n", millis(),
							server->url(), client->id(),
							(info->message_opcode == WS_TEXT) ?
									"text" : "binary");
				printf("%lu ws[%s][%u] frame[%u] start[%llu]\n", millis(),
						server->url(), client->id(), info->num, info->len);
			}

			printf("%lu ws[%s][%u] frame[%u] %s[%llu - %llu]: ", millis(),
					server->url(), client->id(), info->num,
					(info->message_opcode == WS_TEXT) ? "text" : "binary",
					info->index, info->index + len);
			if (info->message_opcode == WS_TEXT) {
				printf("%s\n", (char*) data);
			} else {
				for (size_t i = 0; i < len; i++) {
					printf("%02x ", data[i]);
				}
				printf("\n");
			}

			if ((info->index + len) == info->len) {
				printf("%lu ws[%s][%u] frame[%u] end[%llu]\n", millis(),
						server->url(), client->id(), info->num, info->len);
				if (info->final) {
					printf("%lu ws[%s][%u] %s-message end\n", millis(),
							server->url(), client->id(),
							(info->message_opcode == WS_TEXT) ?
									"text" : "binary");
					/*
					 if(info->message_opcode == WS_TEXT)
					 client->text("I got your text message");
					 else
					 client->binary("I got your binary message");
					 */
				}
			}
		}
	}
}

String processor(const String& var) {
	if (var == "encoder1_value")
		return F("Hello world!");
	return String();
}

bool checkNoApFoundCritical() {
	if (NO_AP_FOUND_count >= 5) {
		WiFi.mode(WIFI_AP);
		if (WiFi.softAP(softAP_ssid, softAP_password)) {
			Serial.println("Wait 100 ms for AP_START...");
			delay(100);
			Serial.println("");
			IPAddress Ip(192, 168, 1, 1);
			IPAddress NMask(255, 255, 255, 0);
			WiFi.softAPConfig(Ip, Ip, NMask);
			IPAddress myIP = WiFi.softAPIP();
			Serial.printf("Network %s is running.\n", softAP_ssid);
			Serial.print("AP IP address: ");
			Serial.println(myIP);
		}
		return true;
	}
	return false;
}

void IRAM_ATTR handleIntCapSense() {
	Serial.println("Capsense interrupt!");
	cap_reading = fdc2212.getReading();
}

void waitForIp() {
	NO_AP_FOUND_count = 0;
	while ((WiFi.status() != WL_CONNECTED) && NO_AP_FOUND_count < 7) {
		Serial.print("MAC: ");
		Serial.println(WiFi.macAddress());

		vTaskDelay(1000 / portTICK_PERIOD_MS);
		Serial.print("SSID: ");
		Serial.println(ssid);
		Serial.print(" password: ");
		Serial.print(password);
		Serial.print(" status: ");
		Serial.println(WiFi.status());

		Serial.print("no ap count count: ");
		Serial.println(NO_AP_FOUND_count);
		//if(checkNoApFoundCritical())
		//  break;
		NO_AP_FOUND_count = NO_AP_FOUND_count + 1;
	}
	if (WiFi.status() != WL_CONNECTED) {
		WiFi.mode(WIFI_AP);
		if (WiFi.softAP(softAP_ssid, softAP_password)) {
			Serial.println("Wait 100 ms for AP_START...");
			delay(100);
			Serial.println("");
			//IPAddress Ip(192, 168, 1, 8);
			//IPAddress NMask(255, 255, 255, 0);
			//WiFi.softAPConfig(Ip, Ip, NMask);
			IPAddress myIP = WiFi.softAPIP();
			Serial.println("Network " + String(softAP_ssid) + " is running.");
			Serial.print("AP IP address: ");
			Serial.println(myIP);
		}
	}

	Serial.print("status: ");
	Serial.println(WiFi.status());

	Serial.print("WiFi local IP: ");
	Serial.println(WiFi.localIP());
}

void blink(int i) {
	for (int j = 0; j < i; j++) {
		digitalWrite(LED_PIN, HIGH);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		digitalWrite(LED_PIN, LOW);
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void setup() {
	Serial.print("Baud rate: 115200");
	Serial.begin(115200);
	Serial.print("ESP ChipSize:");
	Serial.println(ESP.getFlashChipSize());

	if (nvs_flash_init() != ESP_OK) {
		Serial.println("Flash init FAILED!");
	} else
		Serial.println("Flash init OK.");

	pinMode(LED_PIN, OUTPUT);
	pinMode(SS1, OUTPUT);     // Slave select first gate driver
	pinMode(SS2, OUTPUT);     // Slave select second gate driver

	vTaskDelay(3 / portTICK_PERIOD_MS);
	blink(2);

	digitalWrite(SS1, HIGH);   // deselect gate driver 1 - CS to HIGH
	digitalWrite(SS2, HIGH);   // deselect gate driver 2 - CS to HIGH

	pinMode(GATEDRIVER_PIN, OUTPUT); //
	digitalWrite(GATEDRIVER_PIN, LOW);  //disable gate drivers

	//initialise vspi with default pins
	Serial.println("initialise vspi with default pins 1...");
	vspi = new SPIClass(VSPI);
	// VSPI - SCLK = 18, MISO = 19, MOSI = 23, SS = 5
	// begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);

	pinMode(ROTARY_ENCODER2_A_PIN, INPUT_PULLUP);
	pinMode(ROTARY_ENCODER2_B_PIN, INPUT_PULLUP);
	pinMode(ROTARY_ENCODER1_A_PIN, INPUT_PULLUP);
	pinMode(ROTARY_ENCODER1_B_PIN, INPUT_PULLUP);

	pinMode(19, INPUT_PULLUP);
	pinMode(18, OUTPUT);
	pinMode(23, OUTPUT);
	vspi->begin(18, 19, 23, -1);
	vspi->setDataMode(SPI_MODE1);
	vspi->setHwCs(false);

	Serial.println("initialise vspi with default pins 3...");

	delay(10);
	Serial.println("initialise ledc...");
	ledcSetup(LEDC_CHANNEL_0, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_1, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_2, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_3, 20000, LEDC_RESOLUTION);
	ledcAttachPin(PWM1_PIN, LEDC_CHANNEL_0);
	ledcAttachPin(PWM2_PIN, LEDC_CHANNEL_1);
	ledcAttachPin(PWM3_PIN, LEDC_CHANNEL_2);
	ledcAttachPin(PWM4_PIN, LEDC_CHANNEL_3);

	Serial.println("load saved wifi settings...");
	preferences.begin("settings", false);
	//ssid = preferences.getString("wifi_ssid", "null");
	ssid = "null";
	password = preferences.getString("wifi_password", "null");
	password = "null";
	if (ssid.equals("null")) {
		ssid ="AndroidAP";
		//ssid = "AsusKZ";
		password = "Doitman1";
	}

	String pid_str = preferences.getString("pid", "null");
	if (!pid_str.equals("null")) {
		Serial.printf("PID from flash: %s\n", pid_str.c_str());
		setPidsFromString(pid_str);
	} else {
		Serial.printf("no PID from flash.\n");
		pid1.setOutputRampRate(10);
		pid2.setOutputRampRate(10);
		pid1.setPID(70, 40, 10, 0);
		pid2.setPID(70, 40, 10, 0);
	}

	int32_t outputMin_ = preferences.getInt("outputMin1", -100000);
	int32_t outputMax_ = preferences.getInt("outputMax1", -100000);
	if (outputMin_ != -100000 && outputMax_ != -100000) {
		pid1.setOutputLimits(outputMin_, outputMax_);
		Serial.println("Load changed outputmin1 & outputmax1 settings.");
	} else {
		pid1.setOutputLimits(-pwmValueMax, pwmValueMax);
		Serial.println("Missing outputmin1 & outputmax1 settings from flash.");
	}

	outputMin_ = preferences.getInt("outputMin2", -100000);
	outputMax_ = preferences.getInt("outputMax2", -100000);
	if (outputMin_ != -100000 && outputMax_ != -100000) {
		pid2.setOutputLimits(outputMin_, outputMax_);
		Serial.println("Load changed outputmin1 & outputmax1 settings.");
	} else {
		pid2.setOutputLimits(-pwmValueMax, pwmValueMax);
		Serial.println("Missing outputmin1 & outputmax1 settings from flash.");
	}

	int32_t stop1_top_ = preferences.getInt("stop1_top", -100000);
	int32_t stop1_bottom_ = preferences.getInt("stop1_bottom", -100000);
	if (stop1_top_ != -100000) {
		stop1_top = stop1_top_;
		stop2_top = stop1_top;
	}
	if (stop1_bottom_ != -100000) {
		stop1_bottom = stop1_bottom_;
		stop2_bottom = stop1_bottom;
	}
	Serial.println("load saved wifi settings...Done.");
	preferences.end();

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Serial.print("WiFi lost connection. Reason: ");
		Serial.println(info.disconnected.reason);

		String msg="";
		msg.concat(info.disconnected.reason);
		Serial.printf("Reason: %s",msg.c_str());
		if(msg.indexOf("201")>=0) {
			NO_AP_FOUND_count=NO_AP_FOUND_count+1;
			checkNoApFoundCritical();
		}
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

	Serial.println("Configuring adc1");
	adc1_config_width(ADC_WIDTH_BIT_10);
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); //ADC_ATTEN_DB_11 = 0V...3,6V
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); //ADC_ATTEN_DB_11 = 0V...3,6V

	/*
	 if(!ssid.equals(""))
	 {
	 WiFi.mode(WIFI_AP_STA);
	 }
	 else
	 {
	 WiFi.mode(WIFI_AP);
	 }
	 */
	/*
	 WiFi.mode(WIFI_AP_STA);
	 */
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid.c_str(), password.c_str());
	waitForIp();

	Serial.println("Config ntp time...");
	configTzTime(TZ_INFO2, NTP_SERVER0, NTP_SERVER1, NTP_SERVER2);

	time(&now);
	localtime_r(&now, &info);

	Serial.println(&info, "%Y-%m-%d %H:%M:%S");
	Serial.println("Config ntp time...DONE.");

	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		Serial.println("/");
		Serial.println("redirecting to /index.html");
		request->redirect("/index.html");
	});

	server.onNotFound([](AsyncWebServerRequest *request) {
		request->send(404);
	});

	if (!SPIFFS.begin()) {
		Serial.println("SPIFFS Mount Failed");
	}

	listDir(SPIFFS, "/", 0);

	ws.onEvent(onEvent);
	server.addHandler(&ws);

	server.serveStatic("/", SPIFFS, "/").setCacheControl("max-age=600").setDefaultFile(
			"index.html"); //.setTemplateProcessor(processor);

	server.begin();
	ArduinoOTA.begin();
	// OTA callbacks
	ArduinoOTA.onStart([]() {
		// Clean SPIFFS
			pidEnabled = false;
			pwm1=0;
			pwm2=0;

			SPIFFS.end();

			//jsonReporter.detach();
			vTaskSuspend(reportJsonTask);
			//vTaskSuspend(i2cTask);
			//mover.detach();

			// Advertise connected clients what's going on
			ws.textAll("OTA Update Started");

			// Disable client connections
			ws.enable(false);

			// Close them
			ws.closeAll();
		});

	xTaskCreatePinnedToCore(reportJson,  // Task function.
			"reportJsonTask",            // String with name of task.
			30000,                       // Stack size in words.
			NULL,                        // Parameter passed as input of the task
			tskIDLE_PRIORITY,            // Priority of the task.
			&reportJsonTask,             // Task handle.
			1);                          // core number
	esp_task_wdt_add(reportJsonTask);

	/*
	 xTaskCreatePinnedToCore(
	 i2cTask_func,                // Task function.
	 "i2cTask",                   // String with name of task.
	 30000,                       // Stack size in words.
	 NULL,                        // Parameter passed as input of the task
	 tskIDLE_PRIORITY,            // Priority of the task.
	 &i2cTask,                    // Task handle.
	 1);                          // core number
	 */
	rotaryEncoder1.setBoundaries(-10000, 10000, false);
	rotaryEncoder2.setBoundaries(-10000, 10000, false);

	preferences.begin("settings", true);
	encoder1_value = preferences.getInt("encoder1_value");
	encoder2_value = preferences.getInt("encoder2_value");

	rotaryEncoder1.reset(encoder1_value);
	Serial.println(rotaryEncoder1.readEncoder());
	rotaryEncoder2.reset(encoder2_value);
	Serial.println(rotaryEncoder2.readEncoder());

	/*
	 if(abs(preferences.getInt("target1") - encoder1_value)>100)
	 target1 = (double)encoder1_value;
	 else
	 */
	target1 = (double) preferences.getInt("target1");

	/*
	 if(abs(preferences.getInt("target2") - encoder2_value)>100)
	 target2 = (double)encoder2_value;
	 else
	 */
	target2 = (double) preferences.getInt("target2");

	Serial.print("encoder1_value: ");
	Serial.print(encoder1_value);
	Serial.print(" ");
	Serial.print(rotaryEncoder1.readEncoder());
	Serial.print(" target1: ");
	Serial.println(target1);

	Serial.print("encoder2_value: ");
	Serial.print(encoder2_value);
	Serial.print(" ");
	Serial.print(rotaryEncoder2.readEncoder());
	Serial.print(" target2: ");
	Serial.println(target2);

	preferences.end();

	/*
	 xTaskCreatePinnedToCore(
	 Task1,                  // pvTaskCode
	 "Workload1",            // pcName
	 4096,                   // usStackDepth
	 NULL,                   // pvParameters
	 1,                      // uxPriority
	 &TaskA,                 // pxCreatedTask
	 0);                     // xCoreID
	 esp_task_wdt_add(TaskA);
	 */

	Serial.println("Gate driver ON");
	digitalWrite(GATEDRIVER_PIN, HIGH);  //enable gate drivers
	Serial.println("Gate driver ON...Done.");

	//mover.attach_ms(10, move);

	Serial.println("Setting up FDC2212...");
	fdc2212 =
			FDC2212(
					[](const CapacityResponse& response) {
						Serial.printf("capacity triggered %s %ul\n", ((response.status == true)?"ON":"OFF"), response.timeMs);
						return true;
					});
	fdc2212.begin();
	Serial.println("Setting up FDC2212...Done.");

	/*
	 Serial.println("Setting up interrupt for capsense reading...");
	 pinMode(GPIO_NUM_0, INPUT_PULLUP);
	 attachInterrupt(digitalPinToInterrupt(GPIO_NUM_0), handleIntCapSense, RISING);
	 Serial.println("Setting up interrupt for capsense reading...Done.");
	 */

	/*

	 myPing.on(true,[](const AsyncPingResponse& response){
	 IPAddress addr(response.addr); //to prevent with no const toString() in 2.3.0
	 if (response.answer)
	 Serial.printf("%d bytes from %s: icmp_seq=%d ttl=%d time=%d ms\n", response.size, addr.toString().c_str(), response.icmp_seq, response.ttl, response.time);
	 else
	 Serial.printf("no answer yet for %s icmp_seq=%d\n", addr.toString().c_str(), response.icmp_seq);
	 return false; //do not stop
	 });

	 myPing.on(false,[](const AsyncPingResponse& response){
	 IPAddress addr(response.addr); //to prevent with no const toString() in 2.3.0
	 Serial.printf("total answer from %s sent %d recevied %d time %d ms\n",addr.toString().c_str(),response.total_sent,response.total_recv,response.total_time);
	 if (response.mac)
	 Serial.printf("detected eth address " MACSTR "\n",MAC2STR(response.mac->addr));

	 if(WiFi.getMode() != WIFI_MODE_AP)
	 {
	 WiFi.mode(WIFI_AP);
	 if(WiFi.softAP(softAP_ssid, softAP_password))
	 {
	 Serial.println("Wait 100 ms for AP_START...");
	 delay(100);
	 Serial.println("");
	 IPAddress Ip(192, 168, 1, 1);
	 IPAddress NMask(255, 255, 255, 0);
	 WiFi.softAPConfig(Ip, Ip, NMask);
	 IPAddress myIP = WiFi.softAPIP();
	 Serial.println("Network " + String(softAP_ssid) + " is running.");
	 Serial.print("AP IP address: ");
	 Serial.println(myIP);
	 }
	 }

	 return true;
	 });
	 */

	/*
	 tmr = xTimerCreate("MyTimer", pdMS_TO_TICKS(interval), pdTRUE, ( void * )id, &timerCallBack);
	 if( xTimerStart(tmr, 10 ) != pdPASS ) {
	 printf("Timer start error");
	 }
	 */
	esp_task_wdt_init(2, false);

	blink(5);

}

long i;
void loop() {
	i++;
	ArduinoOTA.handle();

	/*
	 if(i % 10000 == 0)
	 {
	 fdc2212.getReading();
	 Serial.printf("FDC2212 read: %du\n", fdc2212.reading);
	 }
	 */
	/*
	 if(i % 500 == 0)
	 {
	 //Serial.printf("FDC2212 read: %du\n", cap_reading);
	 addr.fromString("google.com");
	 myPing.begin(addr)
	 }
	 */

	esp_task_wdt_reset();
	vTaskDelay(500 / portTICK_PERIOD_MS);
	yield();
	//Serial.println("Looping.");

	if (i % 1000 == 0) {
		Serial.print("_async_service_task i=");
		Serial.println(i);
	}
}

void move() {
	// MOTOR1
	if (shouldPwm_M1_left == 1 && shouldStopM1 != 1) {
		if (pwm1 <= (pwmValueMax - pwmDelta)) {
			pwm1 = pwm1 + pwmDelta;
		}
	} else if (shouldPwm_M1_right == 1 && shouldStopM1 != 1) {
		if (pwm1 >= (-pwmValueMax + pwmDelta)) {
			pwm1 = pwm1 - pwmDelta;
		}
	} else if (shouldStopM1 == 1) {
		if (pwm1 > 0)
			pwm1 = pwm1 - pwmDelta;
		else if (pwm1 < 0)
			pwm1 = pwm1 + pwmDelta;
		else
			shouldStopM1 = 0;
	}

	// MOTOR2
	if (shouldPwm_M2_left == 1 && shouldStopM2 != 1) {
		if (pwm2 <= (pwmValueMax - pwmDelta)) {
			pwm2 = pwm2 + pwmDelta;
		}
	} else if (shouldPwm_M2_right == 1 && shouldStopM2 != 1) {
		if (pwm2 >= (-pwmValueMax + pwmDelta)) {
			pwm2 = pwm2 - pwmDelta;
		}
	} else if (shouldStopM2 == 1) {
		if (pwm2 > 0)
			pwm2 = pwm2 - pwmDelta;
		else if (pwm2 < 0)
			pwm2 = pwm2 + pwmDelta;
		else
			shouldStopM2 = 0;
	}

}
