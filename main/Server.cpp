//  python /home/klemen/esp/arduino-esp32/tools/espota.py -I 192.168.43.21 -i 192.168.43.96 -p 3232 -P 3232 -f /home/klemen/esp/hello_world/build/hello-world.bin -d
// platformio run --target uploadfs
// C:\Users\klemen\Dropbox\Voga\BleVogaLifter-esp32-DRV8703Q>c:\Python27\python.exe c:\Users\klemen\.platformio\packages\framework-arduinoespressif32\tools\esptool.py --chip esp32 --port COM3 --baud 115200 --before default_reset --after hard_reset erase_flash
// https://github.com/thehookup/ESP32_Ceiling_Light/blob/master/GPIO_Limitations_ESP32_NodeMCU.jpg
// Need to test: VL53L0X
// https://esp32.com/viewtopic.php?f=13&t=2525#p12056

// cd ~/esp/openocd-esp32
// .\bin\openocd -l out.txt -d3 -s share/openocd/scripts -f interface/ftdi/esp32_devkitj_v1.cfg -f board/esp32-wrover.cfg
// .\bin\openocd -s share/openocd/scripts -f interface/ftdi/esp32_devkitj_v1.cfg -f board/esp32-wrover.cfg

// requires to change #define ASYNC_MAX_ACK_TIME 150000 in AsyncTCP.h
// requires to change #define WS_MAX_QUEUED_MESSAGES 255 in AsyncWebSocket.h

/*handling uploading firmware file */
/*
 To upload through terminal you can use: curl -F "image=@build/DoubleLifter.bin" esp32_door.local/update
 curl -F "image=@build/DoubleLifter.bin" 86.61.7.75/update
 curl -F "image=@build/DoubleLifter.bin" http://192.168.1.7:81/update --progress-bar --verbose
 curl -F "image=@build/DoubleLifter.bin" http://192.168.43.165:81/update --progress-bar --verbose
 curl --verbose --progress-bar -T "./build/DoubleLifter.bin" "http://192.168.1.7:81/update" | tee /dev/null
 */
#include <esp_heap_caps.h>
#include "esp_heap_trace.h"
#include <WiFi.h>
#include <FS.h>

#include "SPIFFS.h"

#include <Update.h>
#include <ESPmDNS.h>
#include <SPI.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>

//#define arduinoWebserver
#ifdef arduinoWebserver
#include <WebServer.h>
#include <WebSocketsServer.h>
#endif
#ifndef arduinoWebserver
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#endif

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

/*SPI Includes*/
#include "driver/spi_master.h"
#include "iot_lcd.h"
#include "Adafruit_GFX.h"
#include "image.h"

#include "FreeSans9pt7b.h"
//#include "unity.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"

#include "debug/lwip_debug.h"
#include "lwip/debug.h"
#include "lwip/stats.h"

#define enableCapSense 1
#define enablePwm 1
#define enableTaskManager 1
#define enableEncSaver 1
bool enableLcd = true;
bool enableMover = false;
bool enableLed = false;
bool shouldReboot = false;

#if enableTaskManager == 1
	#include "Taskmanager.h"
#endif
#if enableEncSaver == 1
	#include "encoderSaver.h"
#endif

#define freeheap heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM

char ptrTaskList[250];
const char* TAG = "DoubleLifter";
bool shouldSendJson = false;
//IRAM_ATTR String getJsonString();

//AsyncUDP udp;
//int udp_port = 1234;

WiFiUDP ntpClient;

static CEspLcd* lcd_obj = NULL;
static int lcd_y_pos = 0;
uint32_t lastWsClient = -1;
float timeH;

// jtag pins: 15, 12 13 14

static int taskManagerCore = 0;
static int encoderSaverCore = 0;

const char* hostName = "esp32_door";
int jsonReportIntervalMs = 100;
int capSenseIntervalMs = 50;
int moverIntervalMs = 50;
int loopIntervalMs = 500;
int encoderSaverIntervalMs = 500;

bool restartNow = false;

long previousMs = 0;

String ssid;
String password;
Preferences preferences;
bool reportingJson = false;

String coredumpStr = "";

const char softAP_ssid[] = "MLIFT";
const char softAP_password[] = "Doitman1";

#ifndef arduinoWebserver
AsyncWebServer server(81);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
#endif
#ifdef arduinoWebserver
WebServer server(81);
WebSocketsServer ws = WebSocketsServer(82);
#endif

#if enableCapSense == 1
#include "FDC2212.h"
FDC2212 fdc2212;
#endif

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

int32_t encoder1_value;
int32_t encoder2_value;
//Ticker mover;
//Ticker jsonReporter;
TaskHandle_t TaskA;
TaskHandle_t TaskMan;
TaskHandle_t TaskEncSaver;
TaskHandle_t TaskLoop;
TaskHandle_t reportJsonTask;
//TaskHandle_t i2cTask;

volatile double output1, output2;
volatile double target1, target2;
double target1_read, target2_read;
volatile bool pidEnabled = true;

AiEsp32RotaryEncoder rotaryEncoder2 = AiEsp32RotaryEncoder(
ROTARY_ENCODER2_A_PIN, ROTARY_ENCODER2_B_PIN, -1, -1);
AiEsp32RotaryEncoder rotaryEncoder1 = AiEsp32RotaryEncoder(
ROTARY_ENCODER1_A_PIN, ROTARY_ENCODER1_B_PIN, -1, -1);

MiniPID pid1 = MiniPID(0.0, 0.0, 0.0);
MiniPID pid2 = MiniPID(0.0, 0.0, 0.0);

//@formatter:off
static String jsonTemplateStr = "{"
		"\"encoder1_value\":%d,"
		"\"encoder2_value\":%d,"
		"\"pwm1\":%d,"
		"\"pwm2\":%d,"
		"\"target1\":%.2f,"
		"\"target2\":%.2f,"
		"\"output1\":%.2f,"
		"\"output2\":%.2f,"
		"\"an1\":%u,"
		"\"an2\":%u,"
		"\"actual_diff\":%.2f,"
		"\"PID1output\":\"p1out=%.2f<br>"
		                  "i1out=%.2f<br>"
											"d1out=%.2f<br>"
											"f1out=%.2f<br>"
											"pos1out=%.2f<br>"
											"setpoint1=%.2f<br>"
											"actual1=%.2f<br>"
											"error1=%.2f<br>"
											"errorSum1=%.2f<br>"
											"maxIOut1=%.2f<br>"
											"maxErr1=%.2f\","
		"\"PID2output\":\"p2out=%.2f<br>"
											"i2out=%.2f<br>"
											"d2out=%.2f<br>"
											"f2out=%.2f<br>"
											"pos2out=%.2f<br>"
											"setpoint2=%.2f<br>"
											"actual2=%.2f<br>"
											"error2=%.2f<br>"
											"errorSum2=%.2f<br>"
											"maxIOut2=%.2f<br>"
											"maxErr2=%.2f\","
		"\"stop1_top\":%u,"
		"\"stop1_bottom\":%u,"
		"\"stop2_top\":%u,"
		"\"stop2_bottom\":%u,"
#if enableCapSense == 1
		"\"cap_reading\":%lu,"
		"\"cap_read_time_ms\":%lu,"
		"\"capfast\":%.2f,"
		"\"capslow\":%.2f,"
#endif
		"\"uptime_h\":%.2f,"
		"\"enablePID\":%d,"
		"\"esp32_heap\":%lu"
		"}\0";
//@formatter:on
char txtToSend[1100] = { };

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

uint32_t cap_reading = 0;

// PID
static String initialPidStr =
		"p=70.00 i=3.00 d=10.00 f=0.00 syn=1 synErr=4.00 ramp=70.00 maxIout=600.00";
static String pid_str;

//AsyncPing myPing;
//IPAddress addr;

/*
 uint32_t idf_wmonitor_coredump_size(void)
 {
 const esp_partition_t *p = coredump_partition();
 return idf_wmonitor_coredump_size_from_partition(p);
 }
 */

//void dbg_lwip_stats_show(void);
void testSpi(int which);
void gdfVdsStatus(int which);
void clearFault();
void setPidsFromString(String str1);
String getToken(String data, char separator, int index);
void sendPidToClient();
void lcd_out(const char *format, ...);
IRAM_ATTR void setJsonString();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
String processInput(const char* input);

TimerHandle_t tmrWs;
void timerCallBack(TimerHandle_t xTimer) {
#ifdef arduinoWebserver
	setJsonString();
	ws.broadcastTXT(txtToSend);
#endif
#ifndef arduinoWebserver
	setJsonString();
	ws.textAll(txtToSend);
#endif
}

#if enableCapSense == 1
TimerHandle_t tmrCapSense;
void timerCapSenseCallBack(TimerHandle_t xTimer) {
	//Serial.printf("getreading \n");
	fdc2212.getReading();
}
#endif

void processWsData(const char *data) {
	if (strncmp(data, "ok", 2) != 0) {
		Serial.printf("processWsData: %s\n", data);
#ifdef arduinoWebserver
		ws.broadcastTXT(processInput(data).c_str());
#else
		String reply = processInput(data);
		if (reply.length() > 0)
			ws.textAll(reply);
#endif
	}
}

TimerHandle_t tmrMover;
void moverCallBack(TimerHandle_t xTimer) {
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
		if (pwm1 > 0) {
			if (pwm1 > pwmDelta)
				pwm1 = pwm1 - pwmDelta;
			else
				pwm1 = 0;
		} else if (pwm1 < 0) {
			if (pwm1 < -pwmDelta)
				pwm1 = pwm1 + pwmDelta;
			else
				pwm1 = 0;
		} else
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
		if (pwm2 > 0) {
			if (pwm2 > pwmDelta)
				pwm2 = pwm2 - pwmDelta;
			else
				pwm2 = 0;
		} else if (pwm2 < 0) {
			if (pwm2 < -pwmDelta)
				pwm2 = pwm2 + pwmDelta;
			else
				pwm2 = 0;
		} else
			shouldStopM2 = 0;
	}
}

String processInput(const char *input) {
	String ret("");
	if (strcmp(input, "status") == 0) {
		status1 = " ";
		status2 = " ";
		testSpi(1);
		ret.concat("Statuses: ");
		ret.concat(status1);
		testSpi(2);
		ret.concat(status2);

		char buf[20];
		sprintf(buf, "motor1_pos %i\n", encoder1_value);
		ret.concat(buf);

		sprintf(buf, "motor2_pos %i\n", encoder2_value);
		ret.concat(buf);

		sprintf(buf, "status1: %s\n", status1.c_str());
		ret.concat(buf);

		sprintf(buf, "status2: %s\n", status2.c_str());
		ret.concat(buf);

		sprintf(buf, "shouldPwm_M1_left: %d\n", shouldPwm_M1_left);
		ret.concat(buf);

		sprintf(buf, "shouldPwm_M1_right: %d\n", shouldPwm_M1_right);
		ret.concat(buf);

		sprintf(buf, "shouldstop_M1: %d\n", shouldStopM1);
		ret.concat(buf);

		sprintf(buf, "shouldPwm_M2_left: %d\n", shouldPwm_M2_left);
		ret.concat(buf);

		sprintf(buf, "shouldPwm_M2_right: %d\n", shouldPwm_M2_right);
		ret.concat(buf);

		sprintf(buf, "shouldstop_M2: %d\n", shouldStopM2);
		ret.concat(buf);
	} else if (strcmp(input, "gdfvdsstatus") == 0) {
		gdfVdsStatus(1);
		gdfVdsStatus(2);

		char buf[20];
		sprintf(buf, "gdfvdsstatus1: %s\n", gdfVds1.c_str());
		ret.concat(buf);

		sprintf(buf, "gdfvdsstatus2: %s\n", gdfVds2.c_str());
		ret.concat(buf);
	} else if (strncmp(input, "clrflt", 6) == 0) {
		clearFault();
		ret.concat("Clear Fault done.");
	} else if (strncmp(input, "pid#", 4) == 0) {
		String input2 = getToken(input, '#', 1);
		setPidsFromString(input2);
		sendPidToClient();

		preferences.begin("settings", false);
		preferences.putString("pid", input2);
		preferences.end();

		ret.concat("Parsing pid done.");
	} else if (strncmp(input, "gCodeCmd", 8) == 0) {
		Serial.printf("Parsing target1=.. target2=... command:%s\n", input);
		String input2 = getToken(input, '#', 1);
		String target1_str = getToken(input2, ' ', 0);
		String target1_duty = getToken(target1_str, '=', 1);
		String target2_str = getToken(input2, ' ', 1);
		String target2_duty = getToken(target2_str, '=', 1);

		target1 = (double) target1_duty.toFloat();
		target2 = (double) target2_duty.toFloat();
		ret.concat("target1= ");
		ret.concat(target1);
		ret.concat("\n");
		ret.concat(" target2= ");
		ret.concat(target2);
		ret.concat("\n");

		preferences.begin("settings", false);
		preferences.putInt("target1", (int) target1);
		preferences.putInt("target2", (int) target2);
		preferences.end();

	} else if (strncmp(input, "enablePid", 9) == 0) {
		pidEnabled = true;
	} else if (strncmp(input, "maxPercentOutput1", 17) == 0) {
		String percent_str = getToken(input, '#', 1);
		setOutputPercent(percent_str, 1);
		preferences.begin("settings", false);
		preferences.putInt("outputMin1", pid1.getMinOutput());
		preferences.putInt("outputMax1", pid1.getMaxOutput());
		preferences.end();
	} else if (strncmp(input, "maxPercentOutput2", 17) == 0) {
		String percent_str = getToken(input, '#', 1);
		setOutputPercent(percent_str, 2);
		preferences.begin("settings", false);
		preferences.putInt("outputMin2", pid2.getMinOutput());
		preferences.putInt("outputMax2", pid2.getMaxOutput());
		preferences.end();
	} else if (strncmp(input, "disablePid", 10) == 0) {
		pidEnabled = false;
	} else if (strncmp(input, "wificonnect", 11) == 0) {
		ssid = getToken(input, ' ', 1);
		password = getToken(input, ' ', 2);

		preferences.begin("settings", false);
		preferences.putString("wifi_ssid", ssid);
		preferences.putString("wifi_password", password);
		preferences.end();

		/*
		 printf("wificonnect ssid: %s, password: %s\n", ssid.c_str(),
		 password.c_str());
		 */
		printf("wificonnect ssid: %s, password: ***\n", ssid.c_str());

		ret.concat("MLIFT restart.");
#ifdef arduinoWebserver
		ws.disconnect();
#endif
#ifndef arduinoWebserver
		ws.enable(false);
#endif

		lcd_out("wificonnect ssid: %s\n", ssid.c_str());

		esp_restart();
	} else if (strncmp(input, "pwm_m1_left", 11) == 0) {
		shouldStopM1 = 0;
		shouldPwm_M1_left = 1;
		shouldPwm_M1_right = 0;
	} else if (strncmp(input, "pwm_m1_right", 12) == 0) {
		shouldStopM1 = 0;
		shouldPwm_M1_left = 0;
		shouldPwm_M1_right = 1;
	} else if (strncmp(input, "stop pwm_m1", 11) == 0) {
		shouldStopM1 = 1;
		shouldPwm_M1_left = 0;
		shouldPwm_M1_right = 0;
	} else if (strncmp(input, "pwm_m2_left", 11) == 0) {
		shouldStopM2 = 0;
		shouldPwm_M2_left = 1;
		shouldPwm_M2_right = 0;
	} else if (strncmp(input, "pwm_m2_right", 12) == 0) {
		shouldStopM2 = 0;
		shouldPwm_M2_left = 0;
		shouldPwm_M2_right = 1;
	} else if (strncmp(input, "stop pwm_m2", 11) == 0) {
		shouldStopM2 = 1;
		shouldPwm_M2_left = 0;
		shouldPwm_M2_right = 0;
	} else if (strncmp(input, "gotop", 5) == 0) {
		target1 = stop1_top;
		target2 = stop1_top;
		Serial.println();
		Serial.print(" target1: ");
		Serial.println(target1);
		Serial.print(" target2: ");
		Serial.println(target2);
		pidEnabled = true;
	} else if (strncmp(input, "gobottom", 8) == 0) {
		target1 = stop1_bottom;
		target2 = stop1_bottom;
		Serial.println();
		Serial.print(" target1: ");
		Serial.println(target1);
		Serial.print(" target2: ");
		Serial.println(target2);
		pidEnabled = true;
	} else if (strncmp(input, "searchtop", 9) == 0
			|| strncmp(input, "searchbottom", 12) == 0) {
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

			if (strcmp(input, "searchtop") == 0) {
				target1 = encoder1_value + deltaSearch;
				target2 = target1;
			} else {
				target1 = encoder1_value - deltaSearch;
				target2 = target1;
			}
			target2 = target1;
			Serial.println();
			Serial.print(" target1: ");
			Serial.println(target1);
			Serial.print(" target2: ");
			Serial.println(target2);

			pidEnabled = true;
			searchTopMilis = millis();
		} else {
			status = "";
			setOutputPercent(previousPercent_str_1, 1);
			setOutputPercent(previousPercent_str_1, 2);
		}
	} else if (strcmp(input, "scan") == 0) {
//vTaskSuspend(reportJsonTask);
//delay(10);
		//xTimerStop(tmrWs, 0);
		Serial.printf("ScanNetworks...Started.\n");
		WiFi.scanDelete();
		WiFi.scanNetworks(true, true, false, 200);
	}

	return ret;
}
#ifdef arduinoWebserver

void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
	switch (type){
	case WStype_DISCONNECTED:
		//USE_SERIAL.printf("[%u] Disconnected!\n", num);
		break;
	case WStype_CONNECTED: {
		//IPAddress ip = webSocket.remoteIP(num);
		//USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
		Serial.print("connected");

		// send message to client
		ws.broadcastTXT("Connected");
		sendPidToClient();
	}
		break;
	case WStype_TEXT:
		if(strcmp((char*) payload, "OK") != 0){
			Serial.printf("Client: [%u] got Text: %s\n", num, payload);
			ws.broadcastTXT(processInput((char*) payload).c_str());
		}
		break;
	case WStype_BIN:
		//USE_SERIAL.printf("[%u] get binary length: %u\n", num, length);

		// send message to client
		// webSocket.sendBIN(num, payload, length);
		break;

	case WStype_ERROR:
	case WStype_FRAGMENT_TEXT_START:
	case WStype_FRAGMENT_BIN_START:
	case WStype_FRAGMENT:
	case WStype_FRAGMENT_FIN:
		break;
	}

}

String getContentTypeGz(String filename){
	if(server.hasArg("download"))
		return "application/octet-stream";
	else if(filename.endsWith(".htm.gz"))
		return "text/html";
	else if(filename.endsWith(".html.gz"))
		return "text/html";
	else if(filename.endsWith(".css.gz"))
		return "text/css";
	else if(filename.endsWith(".js.gz"))
		return "application/javascript";
	else if(filename.endsWith(".png.gz"))
		return "image/png";
	else if(filename.endsWith(".gif.gz"))
		return "image/gif";
	else if(filename.endsWith(".jpg.gz"))
		return "image/jpeg";
	else if(filename.endsWith(".ico.gz"))
		return "image/x-icon";
	else if(filename.endsWith(".xml.gz"))
		return "text/xml";
	else if(filename.endsWith(".pdf.gz"))
		return "application/x-pdf";
	else if(filename.endsWith(".zip.gz"))
		return "application/x-zip";
	else if(filename.endsWith(".gz"))
		return "application/x-gzip";
	return "application/x-gzip";
}

String getContentType(String filename){ // convert the file extension to the MIME type
	if(filename.endsWith(".html"))
		return "text/html";
	else if(filename.endsWith(".css"))
		return "text/css";
	else if(filename.endsWith(".js"))
		return "application/javascript";
	else if(filename.endsWith(".ico"))
		return "image/x-icon";
	return "text/plain";
}

bool handleFileRead(String path){ // send the right file to the client (if it exists)
	Serial.println("handleFileRead: " + path);
	if(path.endsWith("/"))
		path += "index.html"; // If a folder is requested, send the index file
	String contentType = getContentType(path); // Get the MIME type
	String pathWithGz = path + ".gz";
	if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
		if(SPIFFS.exists(pathWithGz)){
			path += ".gz";                          // If the file exists
			contentType = getContentTypeGz(pathWithGz);
		}
		File file = SPIFFS.open(path, "r");                 // Open it
		size_t sent = server.streamFile(file, contentType);                 // And send it to the client
		Serial.printf("Sent %d byte to client.", sent);
		file.close();                 // Then close the file again
		return true;
	}
	Serial.println("\tFile Not Found");
	return false;                     // If the file doesn't exist, return false
}

void handleNotFound(){

	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for(uint8_t i = 0; i < server.args(); i++){
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
}
#endif
#ifndef arduinoWebserver
void wsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
		AwsEventType type, void * arg, uint8_t *data, size_t len) {
	//Serial.printf("%d\n", type);
	if (type == WS_EVT_CONNECT) {
//client connected
		//lcd_out("%lu ws[%s][%u] connect\n", millis(), server->url(), client->id());
		//client->printf("Hello Client %u :)", client->id());
		delay(200);
		//client->ping();
		delay(200);
		sendPidToClient();
		delay(400);
		lastWsClient = client->id();

	} else if (type == WS_EVT_DISCONNECT) {
//client disconnected
		lcd_out("%lu ws[%s][%u] disconnect\n", millis(), server->url(),
				client->id());
	} else if (type == WS_EVT_ERROR) {
//error was received from the other end
		lcd_out("%lu ws[%s][%u] error(%u): %s\n", millis(), server->url(),
				client->id(), *((uint16_t*) arg), (char*) data);
	} else if (type == WS_EVT_PONG) {
//pong message was received (in response to a ping request maybe)
		lcd_out("%lu ws[%s][%u] pong[%u]: %s\n", millis(), server->url(),
				client->id(), len, (len) ? (char*) data : "");
	} else if (type == WS_EVT_DATA) {
//data packet
		AwsFrameInfo * info = (AwsFrameInfo*) arg;
		String msg;
		if (info->final && info->index == 0 && info->len == len) {
//the whole message is in a single frame and we got all of it's data
			//lcd_out("%lu ws[%s][%u] %s-message[%llu]: ", millis(), server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

			//Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

			if (info->opcode == WS_TEXT) {
				for (size_t i = 0; i < info->len; i++) {
					msg += (char) data[i];
				}
			} else {
				char buff[3];
				for (size_t i = 0; i < info->len; i++) {
					sprintf(buff, "%02x ", (uint8_t) data[i]);
					msg += buff;
				}
			}
			//Serial.printf("%s\n", msg.c_str());
			heap_caps_check_integrity_all(true);
			processWsData(msg.c_str());
			heap_caps_check_integrity_all(true);

		} else {
			Serial.printf("multi frames\n");
//message is comprised of multiple frames or the frame is split into multiple packets
			if (info->index == 0) {
				if (info->num == 0)
					Serial.printf("%lu ws[%s][%u] %s-message start\n", millis(),
							server->url(), client->id(),
							(info->message_opcode == WS_TEXT) ?
									"text" : "binary");
				Serial.printf("%lu ws[%s][%u] frame[%u] start[%llu]\n",
						millis(), server->url(), client->id(), info->num,
						info->len);
			}

			Serial.printf("%lu ws[%s][%u] frame[%u] %s[%llu - %llu]: ",
					millis(), server->url(), client->id(), info->num,
					(info->message_opcode == WS_TEXT) ? "text" : "binary",
					info->index, info->index + len);
			if (info->message_opcode == WS_TEXT) {
				Serial.printf("%s\n", (char*) data);
			} else {
				for (size_t i = 0; i < len; i++) {
					Serial.printf("%02x ", data[i]);
				}
				Serial.printf("\n");
			}

			if ((info->index + len) == info->len) {
				Serial.printf("%lu ws[%s][%u] frame[%u] end[%llu]\n", millis(),
						server->url(), client->id(), info->num, info->len);
				if (info->final) {
					Serial.printf("%lu ws[%s][%u] %s-message end\n", millis(),
							server->url(), client->id(),
							(info->message_opcode == WS_TEXT) ?
									"text" : "binary");
				}
			}
		}
	}
}
#endif

void syncTime() {
//lets check the time
	const int NTP_PACKET_SIZE = 48;
	byte ntpPacketBuffer[NTP_PACKET_SIZE];

	IPAddress address;
	WiFi.hostByName("time.nist.gov", address);
	memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);
	ntpPacketBuffer[0] = 0b11100011;   // LI, Version, Mode
	ntpPacketBuffer[1] = 0;     // Stratum, or type of clock
	ntpPacketBuffer[2] = 6;     // Polling Interval
	ntpPacketBuffer[3] = 0xEC;  // Peer Clock Precision
// 8 bytes of zero for Root Delay & Root Dispersion
	ntpPacketBuffer[12] = 49;
	ntpPacketBuffer[13] = 0x4E;
	ntpPacketBuffer[14] = 49;
	ntpPacketBuffer[15] = 52;
	ntpClient.beginPacket(address, 123); //NTP requests are to port 123
	ntpClient.write(ntpPacketBuffer, NTP_PACKET_SIZE);
	ntpClient.endPacket();

	delay(1000);

	int packetLength = ntpClient.parsePacket();
	if (packetLength) {
		if (packetLength >= NTP_PACKET_SIZE) {
			ntpClient.read(ntpPacketBuffer, NTP_PACKET_SIZE);
		}
		ntpClient.flush();
		uint32_t secsSince1900 = (uint32_t) ntpPacketBuffer[40] << 24
				| (uint32_t) ntpPacketBuffer[41] << 16
				| (uint32_t) ntpPacketBuffer[42] << 8 | ntpPacketBuffer[43];
		//Serial.printf("Seconds since Jan 1 1900: %u\n", secsSince1900);
		uint32_t epoch = secsSince1900 - 2208988800UL;
		//Serial.printf("EPOCH: %u\n", epoch);
		uint8_t h = (epoch % 86400L) / 3600;
		uint8_t m = (epoch % 3600) / 60;
		uint8_t s = (epoch % 60);
		Serial.printf("UTC: %02u:%02u:%02u (GMT)\n", h, m, s);
	}
}

void startServer() {
	syncTime();
	MDNS.begin(hostName);

	if (!SPIFFS.begin(true)) {
		lcd_out("SPIFFS Mount Failed");
	} else {
		listDir(SPIFFS, "/", 0);
	}

	ws.onEvent(wsEvent);

#ifdef arduinoWebserver
	server.on("/update", HTTP_POST, [](){
		server.sendHeader("Connection", "close");
		server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
	}, [](){
		HTTPUpload& upload = server.upload();
		if (upload.status == UPLOAD_FILE_START){
			//xTimerStop(tmrWs, 0);
			Serial.printf("Update: %s\n", upload.filename.c_str());
			if (!Update.begin()){ //start with max available size
				Update.printError(Serial);
			}
		} else if (upload.status == UPLOAD_FILE_WRITE){
			if (Update.write(upload.buf, upload.currentSize) != upload.currentSize){
				Update.printError(Serial);
			}
		} else if (upload.status == UPLOAD_FILE_END){
			if (Update.end(true)){ //true to set the size to the current progress
				lcd_out("Update Success: %u\nRebooting...\n", upload.totalSize);
			} else{
				Update.printError(Serial);
				//xTimerStart(tmrWs, 0);
			}
		}
	});

	server.onNotFound([](){                   // If the client requests any URI
				if (!handleFileRead(server.uri()))// send it if it exists
				server.send(404, "text/plain", "404: Not Found");// otherwise, respond with a 404 (Not Found) error
				lcd_out("404: Not Found: %s\n", server.uri().c_str());
			});
	//server.onNotFound(handleNotFound);

	server.on("/", handleRoot);
	ws.begin();
	lcd_out("WebsocketServer started\n");
#else
	// handler for the /update form POST (once file upload finishes)
	server.onFileUpload(
			[](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
				Serial.printf("onFileUpload called, index: %d  len: %d  final: %d\n", index, len, final);
				//shouldSendJson = false;
				uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
				if(0 == index) {
					lcd_out("UploadStart: %s\n", filename.c_str());
					//xTimerStop(tmrWs, 0);
					if(!Update.begin(maxSketchSpace)) {Serial.println("Update begin failure!");}
				}
				if(Update.write(data, len) != len) {
					Update.printError(Serial);
					//shouldSendJson = true;
					//xTimerStart(tmrWs, 0);
				} else {
					Serial.printf("Write: %d bytes\n", len);
				}
				if(final) {
					lcd_out("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
					if (Update.end(true)) {
						lcd_out("Update succesful!");
						restartNow = true;
					} else {
						Update.printError(Serial);
					}
				}
			});

	server.on("/update", HTTP_GET,
			[](AsyncWebServerRequest *request) {
				request->send(200, "text/html", "<form method='POST' action='http://127.0.0.1:81/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
			});

	server.onNotFound([](AsyncWebServerRequest *request) {
		request->send(404);
	});

	server.addHandler(&ws);
	ws.enable(true);
#endif

	server.serveStatic("/index.html", SPIFFS, "/index.html", "max-age=600");
	server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico", "max-age=600");
	server.on("/toggleChartsOn", HTTP_GET, [](AsyncWebServerRequest *request) {
		shouldSendJson = true;
		request->send(200, "text/html", "Toggled OK");
	});
	server.on("/toggleChartsOff", HTTP_GET, [](AsyncWebServerRequest *request) {
		shouldSendJson = false;
		request->send(200, "text/html", "Toggled OK");
	});
	server.on("/gcode", HTTP_GET, [](AsyncWebServerRequest *request) {
		int paramsNr = request->params();
		for(int i=0;i<paramsNr;i++) {

			AsyncWebParameter* p = request->getParam(i);

			if(p->name().equalsIgnoreCase("gcode"))
			{
				Serial.print("Param name: ");
				Serial.println(p->name());

				Serial.print("Param value: ");
				Serial.println(p->value());

				processInput(p->value().c_str());
			}
		}
		request->send(200, "text/html", "Gcode OK");
	});

	server.begin();
	lcd_out("WebServer started.\n");

	MDNS.addService("http", "tcp", 80);
}

String processInput(String input);
void syncTime();

IRAM_ATTR void setJsonString() {

//@formatter:off
		sprintf(txtToSend,
				"{"
				"\"encoder1_value\":%d,"
				"\"encoder2_value\":%d,"
				"\"pwm1\":%d,"
				"\"pwm2\":%d,"
				"\"target1\":%.2f,"
				"\"target2\":%.2f,"
				"\"output1\":%.2f,"
				"\"output2\":%.2f,"
				"\"an1\":%u,"
				"\"an2\":%u,"
				"\"actual_diff\":%.2f,"

#if enableCapSense == 1
		"\"cap_reading\":%zu,"
		"\"cap_read_time_ms\":%lu,"
		"\"capfast\":%.2f,"
		"\"capslow\":%.2f,"
#endif



				"\"esp32_heap\":%zu,"
				"\"uptime_h\":%.2f"

				"}",

				encoder1_value,
				encoder2_value,
				pwm1,
				pwm2,
				target1,
				target2,
				output1,
				output2,
				an1,
				an2,
				(pid1.getActual() - pid2.getActual()),

#if enableCapSense == 1
			cap_reading,
			fdc2212.readTimeMs,
			fdc2212.capFast,
			fdc2212.capSlow,
#endif

				esp_get_free_heap_size(),
				timeH);
//@formatter:on
}

void lcd_out(const char*format, ...) {
	char loc_buf[255];
	char * temp = loc_buf;
	va_list arg;
	va_list copy;
	va_start(arg, format);
	va_copy(copy, arg);
	size_t len = vsnprintf(NULL, 0, format, arg);
	va_end(copy);
	if (len >= sizeof(loc_buf)) {
		temp = new char[len + 1];
		if (temp == NULL) {
			return;
		}
	}
	len = vsnprintf(temp, len + 1, format, arg);

	if (enableLcd) {
		lcd_obj->drawString(loc_buf, 3, lcd_y_pos);
		lcd_y_pos = lcd_y_pos + 10;
		if (lcd_y_pos > 250) {
			lcd_y_pos = 0;
			lcd_obj->fillScreen(COLOR_ESP_BKGD);
		}
	}
	//ESP_LOGI(TAG, "%s", temp);
	Serial.printf("%s\n", temp);

	va_end(arg);
	if (len >= sizeof(loc_buf)) {
		delete[] temp;
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

// takes pid string in form:
// p=40.00 i=2.00 d=0.30 f=0.00 syn=1 synErr=0.00 ramp=50.00 maxIout=1000.00
// p=40.00 i=2.00 d=0.30 f=0.00 syn=0 synErr=0.00 ramp=50.00 maxIout=1000.00
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
	if (f_val.equals("")) {
		f_val = "0.0";
	}

	String syn_str = getToken(input, ' ', 4);
	String syn_val = getToken(syn_str, '=', 1);

	String synerr_str = getToken(input, ' ', 5);
	String synerr_val = getToken(synerr_str, '=', 1);

	String ramp_str = getToken(input, ' ', 6);
	String ramp_val = getToken(ramp_str, '=', 1);
	if (ramp_val.equals("")) {
		ramp_val = "1.0";
	}

	String maxIOut_str = getToken(input, ' ', 7);
	String maxIOut_val = getToken(maxIOut_str, '=', 1);
	if (maxIOut_val.equals("")) {
		maxIOut_val = "1.0";
	}

	pid1.setPID(p_val.toFloat(), i_val.toFloat(), d_val.toFloat(),
			f_val.toFloat());
	pid2.setPID(p_val.toFloat(), i_val.toFloat(), d_val.toFloat(),
			f_val.toFloat());
	pid1.setOutputRampRate(ramp_val.toFloat());
	pid2.setOutputRampRate(ramp_val.toFloat());	//pid1.setOutputFilter(0.01);
//pid2.setOutputFilter(0.01);
	Serial.print("f_val: ");
	Serial.println(f_val.toFloat());
	pid1.setMaxIOutput(maxIOut_val.toFloat());
	pid2.setMaxIOutput(maxIOut_val.toFloat());
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
	pid_str = String("");
	pid_str.concat("{");
	pid_str.concat("\"pid\":");
	pid_str.concat("\"p=");
	pid_str.concat(pid1.getP());
	pid_str.concat(" i=");
	pid_str.concat(pid1.getI());
	pid_str.concat(" d=");
	pid_str.concat(pid1.getD());
	pid_str.concat(" f=");
	pid_str.concat(pid1.getF());
	pid_str.concat(" syn=");
	pid_str.concat(pid1.getSynchronize() ? "1" : "0");
	pid_str.concat(" synErr=");
	pid_str.concat(pid1.getSyncDisabledForErrorSmallerThen());
	pid_str.concat(" ramp=");
	pid_str.concat(pid1.getRampRate());
	pid_str.concat(" maxIout=");
	pid_str.concat(pid1.getMaxIOutput());
	pid_str.concat("\",");
	pid_str.concat("\"maxPercentOutput\":");
	pid_str.concat((int) (ceil(pid1.getMaxOutput() / pwmValueMax * 100.0)));
	pid_str.concat("}");

#ifdef arduinoWebserver
	ws.broadcastTXT(pid_str.c_str());
#endif // arduinoWebserver
#ifndef arduinoWebserver
	ws.textAll(pid_str.c_str());
#endif

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

void listDir(fs::FS & fs, const char * dirname, uint8_t levels) {
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
	digitalWrite(SS1, LOW);	  	// SPI WRITE
	vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
	byte data_read = B00000000;	  	// WRITE OPERATION
	byte data_address = B00010000;	  	// ADDRES 02x MAIN REGISTER
	byte data = data_read | data_address;

	uint16_t data_int = data << 8 | B00000001;	  	// B00000001 ... CLR_FLT
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
	uint8_t FAULT_FAULT = B1 << 7;// FAULT R 0b Logic OR of the FAULT status register excluding the OTW bit
	uint8_t FAULT_WDFLT = B1 << 6;	  	// WDFLT R 0b Watchdog time-out fault
	uint8_t FAULT_GDF = B1 << 5;// GDF R 0b Indicates gate drive fault condition
	uint8_t FAULT_OCP = B1 << 4;// OCP R 0b Indicates VDS monitor overcurrent fault condition
	uint8_t FAULT_VM_UVFL = B1 << 3;// VM_UVFL R 0b Indicates VM undervoltage lockout fault condition
	uint8_t FAULT_VCP_UVFL = B1 << 2;// VCP_UVFL R 0b Indicates charge-pump undervoltage fault condition
	uint8_t FAULT_OTSD = B1 << 1;// OTSD R 0b Indicates overtemperature shutdown
	uint8_t FAULT_OTW = B1 << 0;// OTW R 0b Indicates overtemperature warning
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
	byte data_read = B10000000;	  // READ OPERATION
	byte data_address = B00010000;	  // ADDRES 02x MAIN REGISTER
	byte data = data_read | data_address;
	byte lowbyte = B0;
	uint16_t data_int = data << 8 | lowbyte;

	uint16_t reply = vspi->transfer16(data_int);// should return 0x18 B00011000
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

		data_read = B10000000;	  // READ OPERATION
		data_address = B00000000;	  // ADDRES 0x FAULT REGISTER
		data = data_read | data_address;
		lowbyte = B0;
		data_int = data << 8 | lowbyte;

		reply = vspi->transfer16(data_int);	  // should return 0x18
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

	byte data_read = B10000000;	  // READ OPERATION
	byte data_address = B00001000;// ADDRES 01x VDS and GDF Status Register Name
	byte data = data_read | data_address;
	byte lowbyte = B0;
	uint16_t data_int = data << 8 | lowbyte;

	uint16_t reply = vspi->transfer16(data_int);// should return GDF and VDS statuses
	vspi->endTransaction();
	if (which == 1)
		digitalWrite(SS1, HIGH);
	else
		digitalWrite(SS2, HIGH);
	usleep(1);
	String ret = "";

// fault bytes:
	uint8_t H2_GDF = B1 << 7;// Gate drive fault on the high-side FET of half bridge 2
	uint8_t L2_GDF = B1 << 6;// Gate drive fault on the low-side FET of half bridge 2
	uint8_t H1_GDF = B1 << 5;// Gate drive fault on the high-side FET of half bridge 1
	uint8_t L1_GDF = B1 << 4;// Gate drive fault on the low-side FET of half bridge 1
	uint8_t H2_VDS = B1 << 3;// VDS monitor overcurrent fault on the high-side FET of half bridge 2
	uint8_t L2_VDS = B1 << 2;// VDS monitor overcurrent fault on the low-side FET of half bridge 2
	uint8_t H1_VDS = B1 << 1;// VDS monitor overcurrent fault on the high-side FET of half bridge 1
	uint8_t L1_VDS = B1 << 0;// VDS monitor overcurrent fault on the low-side FET of half bridge 1

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

/*
 void IRAM_ATTR handleIntCapSense(){
 Serial.println ("Capsense interrupt!");
 cap_reading = fdc2212.getReading ();
 }
 */

void waitForIp() {
	NO_AP_FOUND_count = 0;

	WiFi.mode(WIFI_STA);
	WiFi.enableIpV6();
	WiFi.setTxPower(WIFI_POWER_19_5dBm);
	WiFi.begin(ssid.c_str(), password.c_str());
	WiFi.setSleep(false);

	while ((WiFi.status() != WL_CONNECTED) && NO_AP_FOUND_count < 7) {
		Serial.print("MAC: ");
		Serial.println(WiFi.macAddress());
		lcd_out("WaitForIp delay 1s.\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		Serial.print("SSID: ");
		Serial.print(ssid);
		Serial.print(" password: ***");
		//Serial.print(password);
		Serial.print(" status: ");
		Serial.println(WiFi.status());

		Serial.print("no ap count count: ");
		Serial.println(NO_AP_FOUND_count);
//if(checkNoApFoundCritical())
//  break;
		NO_AP_FOUND_count = NO_AP_FOUND_count + 1;
	}
	if (WiFi.status() != WL_CONNECTED) {
		lcd_out("Could not connect... Entering in AP mode.\n");
		WiFi.mode(WIFI_AP);
		if (WiFi.softAP(softAP_ssid, softAP_password)) {
			Serial.println("Wait 100 ms for AP_START...");
			delay(100);
			//IPAddress Ip(192, 168, 1, 8);
			//IPAddress NMask(255, 255, 255, 0);
			//WiFi.softAPConfig(Ip, Ip, NMask);
			IPAddress myIP = WiFi.softAPIP();
			lcd_out((String(softAP_ssid) + " is running.\n").c_str());
			lcd_out((myIP.toString() + "\n").c_str());
			Serial.print("AP IP address: ");
			Serial.println(myIP);

			tcpip_adapter_ip_info_t ip_info;
			char* str2;
			ESP_ERROR_CHECK(
					tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));
			str2 = inet_ntoa(ip_info);
			String buf("WiFi AP IP: ");
			buf.concat(str2);
			buf.concat("\n");
			lcd_out(buf.c_str());

			startServer();
		}
	} else {
		tcpip_adapter_ip_info_t ip_info;
		char* str2;
		ESP_ERROR_CHECK(
				tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
		str2 = inet_ntoa(ip_info);
		String buf("WiFi STA IP: ");
		buf.concat(str2);
		buf.concat("\n");
		lcd_out(buf.c_str());
	}

	Serial.print("status: ");
	Serial.println(WiFi.status());
	Serial.print("WiFi local IP: ");
	Serial.println(WiFi.localIP());

	Serial.print("WiFi local IPv6: ");
	Serial.println(WiFi.localIPv6());

}

void blink(int i) {
	if (enableLed) {
		for (int j = 0; j < i; j++) {
			digitalWrite(LED_PIN, HIGH);
			vTaskDelay(50 / portTICK_PERIOD_MS);
			digitalWrite(LED_PIN, LOW);
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
	}
}

extern "C" void esp_draw() {
	/*Initilize ESP32 to scan for Access points*/
	nvs_flash_init();
	/*
	 tcpip_adapter_init();
	 wifi_event_group = xEventGroupCreate();
	 ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	 wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	 ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	 ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	 ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	 ESP_ERROR_CHECK( esp_wifi_start() );
	 */
	/*Initialize LCD*/
	lcd_conf_t lcd_pins = { .lcd_model = LCD_MOD_AUTO_DET, .pin_num_miso =
			GPIO_NUM_25, .pin_num_mosi = GPIO_NUM_23,
			.pin_num_clk = GPIO_NUM_19, .pin_num_cs = GPIO_NUM_22, .pin_num_dc =
					GPIO_NUM_21, .pin_num_rst = GPIO_NUM_18, .pin_num_bckl =
					GPIO_NUM_5, .clk_freq = 26 * 1000 * 1000,
			.rst_active_level = 0, .bckl_active_level = 0,
			.spi_host = HSPI_HOST, .init_spi_bus = true, };

	if (lcd_obj == NULL) {
		lcd_obj = new CEspLcd(&lcd_pins);
	}
	printf("lcd id: 0x%08x\n", lcd_obj->id.id);

	lcd_obj->setRotation(2);
	lcd_obj->fillScreen(COLOR_ESP_BKGD);
	lcd_obj->setTextSize(1);
	lcd_obj->drawBitmap(0, 0, esp_logo, 137, 26);

	lcd_obj->setTextColor(COLOR_GREEN, COLOR_ESP_BKGD);
	lcd_obj->setFont(NULL);

}

//void createReportJsonTask() {
//	xTaskCreatePinnedToCore(reportJson,  // Task function.
//			"reportJsonTask",            // String with name of task.
//			30000,                       // Stack size in words.
//			NULL,                       // Parameter passed as input of the task
//			17,            // Priority of the task.
//			&reportJsonTask,             // Task handle.
//			0);                          // core number
//	esp_task_wdt_add(reportJsonTask);
//}

void printEncoderInfo() {
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
}

/*
 static void idf_wmonitor_start_task(tcpip_adapter_if_t iface) {
 xTaskCreatePinnedToCore(idf_monitor_server_task, "WMONITOR", 4096,
 (void *) iface, tskIDLE_PRIORITY + 1,
 NULL, WIFI_TASK_CORE_ID);
 }


 static void idf_wmonitor_do_coredump_read(int s)
 {
 uint32_t coredump_size = idf_wmonitor_coredump_size();
 uint8_t resp = CMD_COREDUMP_READ;
 coredump_size = htonl(coredump_size);
 xSemaphoreTake(state.socket_sema, portMAX_DELAY);
 write(s, &resp, sizeof(resp));
 write(s, &coredump_size, sizeof(&coredump_size));
 idf_wmonitor_coredump_read(idf_wmonitor_coredump_reader, &s);
 xSemaphoreGive(state.socket_sema);
 }
 */

void setup() {
	Serial.setDebugOutput(true);
	esp_log_level_set("*", ESP_LOG_VERBOSE);
	esp_log_level_set("I2Cbus", ESP_LOG_WARN);
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);//esp_log_level_set("phy_init", ESP_LOG_INFO);
	lcd_out("DoubeLifter START %d", 1);
	Serial.println("Baud rate: 115200");
	Serial.begin(115200);
	Serial.print("ESP ChipSize:");
	Serial.println(ESP.getFlashChipSize());
	lcd_out("Flash INIT\n");
	if (nvs_flash_init() != ESP_OK) {
		lcd_out("Flash init FAILED!\n");
		nvs_flash_init_partition("nvs");
		nvs_flash_init();
	} else
		lcd_out("Flash init OK.\n");

	lcd_out("LED INIT\n");
	if (enableLed)
		pinMode(LED_PIN, OUTPUT);

	vTaskDelay(3 / portTICK_PERIOD_MS);
	lcd_out("Blinking.\n");
	blink(2);

#if enableCapSense == 1
	lcd_out("Setting up FDC2212...\n");
	fdc2212 =
			FDC2212(
					[](const CapacityResponse& response) {
						Serial.printf("capacity triggered %s %ul\n", ((response.status == true)?"ON":"OFF"), response.timeMs);
						return true;
					});
	fdc2212.begin();
	lcd_out("Setting up FDC2212...Done.\n");
#endif

	pinMode(19, INPUT_PULLUP);
	pinMode(18, OUTPUT);
	pinMode(23, OUTPUT);
	lcd_out("VSPI?\n");
	if (vspi != NULL) {
		Serial.println("initialise vspi with default pins 3...");
		vspi->begin(18, 19, 23, -1);
		vspi->setDataMode(SPI_MODE1);
		vspi->setHwCs(false);
	}

	lcd_out("Loading WIFI setting\n");
	preferences.begin("settings", false);
	ssid = preferences.getString("wifi_ssid", "null");
//ssid = "null";
	password = preferences.getString("wifi_password", "null");
//password = "null";
	if (ssid.equals("null")) {
		ssid = "AndroidAP";
//ssid = "AsusKZ";
		password = "Doitman1";
	}
//password = "klemenklemen";
//ssid = "SINTEX";
	lcd_out(String(" ssid:     " + ssid + "\n").c_str());
	lcd_out(String(" pass:     " + password + "\n").c_str());

	pid_str = preferences.getString("pid", "null");
	if (!pid_str.equals("null")) {
		Serial.printf("PID from flash: %s\n", pid_str.c_str());
		setPidsFromString(pid_str);
	} else {
		Serial.printf("no PID from flash.\n");
		Serial.printf("using initial string: %s\n", initialPidStr.c_str());
		setPidsFromString(initialPidStr);
		pid_str = initialPidStr;
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
		lcd_out("Wifi lost connection.\n");

		String msg="";
		msg.concat(info.disconnected.reason);
		lcd_out(String("Reason: " + msg + "\n").c_str());
		if(msg.indexOf("201")>=0) {
			NO_AP_FOUND_count=NO_AP_FOUND_count+1;
			checkNoApFoundCritical();
		}
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		lcd_out("SYSTEM_EVENT_SCAN_DONE");
		int n = WiFi.scanComplete();
		if(n>0)
		{
			String ret;
			for (int i = 0; i < n; ++i) {
				ret.concat("wifi ");
				String wifiData="";
				wifiData.concat(WiFi.SSID(i));
				wifiData.concat(" (");
				wifiData.concat(WiFi.RSSI(i));
				wifiData.concat(") ");
				wifiData.concat(
						(WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ?
						"OPEN" : "PASS");
				ret.concat(wifiData);
				ret.concat("\n");
				lcd_out(String(wifiData + "\n").c_str());
			}

#ifdef arduinoWebserver
			ws.broadcastTXT(ret);
#endif
#ifndef arduinoWebserver
			ws.textAll(ret);
#endif
			//lcd_out("Resume reportJsonTask\n");
			//xTimerStart(tmrWs, 0);
//vTaskResume(reportJsonTask);
		}
		else if (n==0)
		{
			//ws.textAll("wifi No networks found.");
			//vTaskResume(reportJsonTask);
		}

	}, WiFiEvent_t::SYSTEM_EVENT_SCAN_DONE);
//	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
//		lcd_out("SYSTEM_EVENT_STA_GOT_IP\n");
//		lcd_out(String(WiFi.localIPv6().toString()+ "\n").c_str());
//		lcd_out(String(WiFi.softAPIPv6().toString() + "\n").c_str());
//	}, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		startServer();
	}, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		lcd_out("SYSTEM_EVENT_GOT_IP6\n");
		startServer();
	}, WiFiEvent_t::SYSTEM_EVENT_GOT_IP6);
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		WiFi.begin();
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

	waitForIp();			//	wifi_mode_t mode = WiFi.getMode();
//	if (mode == WIFI_MODE_AP) {
//		lcd_out("WIFI_MODE_AP");
//
//	}

	rotaryEncoder1.setBoundaries(0, 30000, false);
	rotaryEncoder2.setBoundaries(0, 30000, false);
	preferences.begin("settings", true);
	encoder1_value = preferences.getInt("encoder1_value", -32000);
	Serial.println("");
	Serial.print("read encoder1_value: ");
	Serial.println(encoder1_value);
	rotaryEncoder1.disable();
	if (encoder1_value != -32000)
		rotaryEncoder1.reset(encoder1_value);
	else {
		encoder1_value = 15000;
		Serial.printf("Resetting encoder1 to: %d\n", encoder1_value);
		rotaryEncoder1.reset(encoder1_value);
		target1 = encoder1_value;
		Serial.println("reset encoder1.");
	}
	rotaryEncoder1.enable();
	encoder2_value = preferences.getInt("encoder2_value", -32000);
	Serial.print("read encoder2_value: ");
	Serial.println(encoder2_value);
	rotaryEncoder2.disable();
	if (encoder2_value != -32000)
		rotaryEncoder2.reset(encoder2_value);
	else {
		encoder2_value = 15000;
		Serial.printf("Resetting encoder2 to: %d\n", encoder2_value);
		rotaryEncoder2.reset(encoder2_value);
		target2 = encoder2_value;
		Serial.println("reset encoder2.");
	}
	rotaryEncoder2.enable();
	Serial.printf("encoder1: %d\n", rotaryEncoder1.readEncoder());
	Serial.printf("encoder2: %d\n", rotaryEncoder2.readEncoder());
	target1_read = (double) preferences.getInt("target1", -15000);
	target2_read = (double) preferences.getInt("target2", -15000);
	preferences.end();
	if (target1_read == -15000) {
		target1 = rotaryEncoder1.readEncoder();
	} else {
		target1 = target1_read;
	}

	if (target2_read == -15000) {
		target2 = rotaryEncoder2.readEncoder();
	} else {
		target2 = target2_read;
	}

	printEncoderInfo();
	Serial.println("ENA");
	esp_err_t errWdtInit = esp_task_wdt_init(5, false);
	if (errWdtInit != ESP_OK) {
		log_e("Failed to init WDT! Error: %d", errWdtInit);
	}

#if enableEncSaver == 1
	lcd_out("Starting encoder saver...");
	Serial.flush();
	xTaskCreatePinnedToCore(encoderSaverTask,			// pvTaskCode
			"EncoderSaver",			// pcName
			4096,			// usStackDepth
			NULL,			// pvParameters
			22,			// uxPriority
			&TaskEncSaver,			// pxCreatedTask
			encoderSaverCore);			// xCoreID
	lcd_out("Starting encoder saver task...Done.\n");
	Serial.flush();
#endif

//mover.attach_ms(10, move);

	/*
	 int id1 = 1;
	 tmrWs = xTimerCreate("MyTimer", pdMS_TO_TICKS(jsonReportIntervalMs), pdTRUE,
	 (void *) id1, &timerCallBack);
	 if (xTimerStart(tmrWs, pdMS_TO_TICKS(100)) != pdPASS) {
	 lcd_out("Timer jsonReport start error.\n");
	 } else {
	 lcd_out("Timer jsonReport started.\n");
	 }
	 Serial.flush();
	 */
#if enableCapSense == 1
	int id2 = 2;
	tmrCapSense = xTimerCreate("MyTimerCapSense",
			pdMS_TO_TICKS(capSenseIntervalMs), pdTRUE, (void *) id2,
			&timerCapSenseCallBack);
	if (xTimerStart(tmrCapSense, pdMS_TO_TICKS(100)) != pdPASS) {
		esp_log_write(ESP_LOG_ERROR, TAG, "Timer capsense start error");
	} else {
		esp_log_write(ESP_LOG_INFO, TAG, "Timer capsense start.");
	}
#endif

	if (enableMover) {
		int id3 = 3;
		tmrMover = xTimerCreate("MyTimerMover", pdMS_TO_TICKS(moverIntervalMs),
		pdTRUE, (void *) id3, &moverCallBack);
		if (xTimerStart(tmrMover, pdMS_TO_TICKS(100)) != pdPASS) {
			esp_log_write(ESP_LOG_ERROR, TAG, "Timer mover start error");
		} else {
			esp_log_write(ESP_LOG_INFO, TAG, "Timer mover start.");
		}
	}

#if enablePwm == 1
	int pidTaskCore = 1;
	lcd_out("Gate driving enable.\n");
	pinMode(GATEDRIVER_PIN, OUTPUT);
	digitalWrite(GATEDRIVER_PIN, HIGH);					//enable gate drivers

	pinMode(SS1, OUTPUT);	// Slave select first gate driver
	pinMode(SS2, OUTPUT);	// Slave select second gate driver

	digitalWrite(SS1, HIGH);	// deselect gate driver 1 - CS to HIGH
	digitalWrite(SS2, HIGH);	// deselect gate driver 2 - CS to HIGH

	digitalWrite(GATEDRIVER_PIN, LOW);		//disable gate drivers

//initialise vspi with default pins
	lcd_out("initialise vspi with default pins 1...\n");
	vspi = new SPIClass(VSPI);
// VSPI - SCLK = 18, MISO = 19, MOSI = 23, SS = 5
// begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);

	lcd_out("encoders\n");
	pinMode(ROTARY_ENCODER2_A_PIN, INPUT_PULLUP);
	lcd_out("encoders 1\n");
	pinMode(ROTARY_ENCODER2_B_PIN, INPUT_PULLUP);
	lcd_out("encoders 2\n");
	pinMode(ROTARY_ENCODER1_A_PIN, INPUT_PULLUP);
	lcd_out("encoders 3\n");
	pinMode(ROTARY_ENCODER1_B_PIN, INPUT_PULLUP);
	lcd_out("encoders 4\n");

	delay(100);
	Serial.println("initialise PWM ...");
	ledcSetup(LEDC_CHANNEL_0, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_1, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_2, 20000, LEDC_RESOLUTION);
	ledcSetup(LEDC_CHANNEL_3, 20000, LEDC_RESOLUTION);
	ledcAttachPin(PWM1_PIN, LEDC_CHANNEL_0);
	ledcAttachPin(PWM2_PIN, LEDC_CHANNEL_1);
	ledcAttachPin(PWM3_PIN, LEDC_CHANNEL_2);
	ledcAttachPin(PWM4_PIN, LEDC_CHANNEL_3);
	delay(100);

	lcd_out("Starting pidTask...");
	Serial.flush();
	xTaskCreatePinnedToCore(Task1,			// pvTaskCode
			"pidTask",			// pcName
			6096,			// usStackDepth
			NULL,			// pvParameters
			1,			    // uxPriority
			&TaskA,			// pxCreatedTask
			pidTaskCore);			// xCoreID
	esp_task_wdt_add(TaskA);
	digitalWrite(GATEDRIVER_PIN, HIGH);			//enable gate drivers
	lcd_out("Starting pidTask...Done.\n");

	Serial.flush();
#endif

#if enableTaskManager == 1
	lcd_out("Starting taskmanager...");
	Serial.flush();
	xTaskCreatePinnedToCore(taskmanageTask,			// pvTaskCode
			"TaskManager",			// pcName
			4096,			// usStackDepth
			NULL,			// pvParameters
			22,			// uxPriority
			&TaskMan,			// pxCreatedTask
			taskManagerCore);			// xCoreID
	esp_task_wdt_add(TaskMan);
	lcd_out("Starting Taskmanager task...Done.\n");
	Serial.flush();
#endif

	blink(5);
	lcd_out("Setup Done.\n");

//printEncoderInfo();
}

/*
 void dbg_lwip_stats_show(void)
 {
 TCP_STATS_DISPLAY();
 UDP_STATS_DISPLAY();
 ICMP_STATS_DISPLAY();
 IGMP_STATS_DISPLAY();
 IP_STATS_DISPLAY();
 IPFRAG_STATS_DISPLAY();
 ETHARP_STATS_DISPLAY();
 LINK_STATS_DISPLAY();
 MEM_STATS_DISPLAY();
 SYS_STATS_DISPLAY();
 IP6_STATS_DISPLAY();
 ICMP6_STATS_DISPLAY();
 IP6_FRAG_STATS_DISPLAY();
 MLD6_STATS_DISPLAY();
 ND6_STATS_DISPLAY();
 ESP_STATS_DISPLAY();
 }
 */

uint32_t previousHeap;
uint64_t mySecond = 0;
uint64_t previousSecond = 0;
long delta;
long start;
void myLoop() {			//ArduinoOTA.handle();

//printEncoderInfo();
//	for(;;){
	mySecond = esp_timer_get_time() / 1000000.0;
	if (((mySecond % 10 == 0) && (previousSecond != mySecond))
			|| (abs(ESP.getFreeHeap() - previousHeap) > 10000)) {
#ifndef arduinoWebserver
		timeH = (float) (esp_timer_get_time() / (1000000.0 * 60.0 * 60.0));
		lcd_out(
				"time[s]: %" PRIu64 " uptime[h]: %.2f core: %d, freeHeap: %u, wsLength: %d\n",
				mySecond, timeH, xPortGetCoreID(), freeheap,
				ws._buffers.length());
#else
			timeH = (float) (esp_timer_get_time() / (1000000.0 * 60.0 * 60.0));
			lcd_out("time[s]: %" PRIu64 " uptime[h]: %.2f core: %d, freeHeap: %u", mySecond, timeH, xPortGetCoreID(), freeheap);
			#endif
		//dbg_lwip_stats_show();
		heap_caps_check_integrity_all(true);
		if (abs(ESP.getFreeHeap() - previousHeap) > 10000)
			previousHeap = ESP.getFreeHeap();
		previousSecond = mySecond;
		previousMs = millis();

		/*
		 vTaskList(ptrTaskList);
		 Serial.println(F("**********************************"));
		 Serial.println(F("Task  State   Prio    Stack    Num"));
		 Serial.println(F("**********************************"));
		 Serial.print(ptrTaskList);
		 Serial.println(F("**********************************"));
		 */

	}
	heap_caps_check_integrity_all(true);

	esp_err_t resetOK = esp_task_wdt_reset();
	if (resetOK != ESP_OK) {
		//lcd_out("Failed reset wdt: err %#03x\n", resetOK);
	}

	if (restartNow) {
		lcd_out("Restarting...\n");
		ESP.restart();
	}

	//if((millis() > (previousMs + jsonReportIntervalMs)) && shouldSendJson){
	if (shouldSendJson) {
		setJsonString();
		//Serial.println(txtToSend);
		//ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
#ifdef arduinoWebserver
			ws.broadcastTXT(txtToSend);
#else
		if (ws.hasClient(lastWsClient)) {
			ws.text(lastWsClient, txtToSend);
		}
		/*
		 uint8_t opcode = WS_TEXT;
		 uint8_t _opcode = opcode & 0x07;
		 bool mask = false;
		 size_t len = sizeof(txtToSend)/sizeof(txtToSend[0]);
		 if(ws.hasClient(lastWsClient)){
		 Serial.printf("Space: %d\n", ((AsyncClient*) ws.client(lastWsClient))->space());
		 size_t sent = webSocketSendFrame((AsyncClient*) ws.client(lastWsClient), true, _opcode, mask, (uint8_t*) txtToSend, len);
		 Serial.printf("Sent len: %d\n", sent);
		 }
		 */
#endif

		//ESP_ERROR_CHECK(heap_trace_stop());
		//heap_trace_dump();
//Serial.printf("texted all: %s\n", txtToSend);

	}

#ifdef arduinoWebserver
		server.handleClient();
		ws.loop();
#endif

	//vTaskDelay(10 / portTICK_PERIOD_MS);
	//optimistic_yield(5);

//	}
}

void loop() {
	vTaskSuspend(NULL);
}

void Loop(void*parameter) {
	myLoop();
}

int id3 = 4;
TimerHandle_t tmr2;
void loopCallBack(TimerHandle_t xTimer) {
	//loop();
	myLoop();
}

extern "C" {
void app_main();
}
void app_main() {
	ESP_ERROR_CHECK(heap_trace_init_standalone ( trace_record , NUM_RECORDS ));

	if (enableLcd == true)
		esp_draw();

	setup();

	/*
	 lcd_out("Starting LoopTask...");
	 Serial.flush();
	 xTaskCreatePinnedToCore(Loop,							// pvTaskCode
	 "MyLoop",							// pcName
	 25000,							// usStackDepth
	 NULL,							// pvParameters
	 16,							// uxPriority
	 &TaskLoop,							// pxCreatedTask
	 taskCore);							// xCoreID
	 esp_task_wdt_add(TaskLoop);
	 lcd_out("Starting LoopTask...Done.\n");
	 Serial.flush();
	 */

	tmr2 = xTimerCreate("MyTimer", pdMS_TO_TICKS(jsonReportIntervalMs), pdTRUE,
			(void *) id3, &loopCallBack);
	if ( xTimerStart ( tmr2 , 100 / portTICK_PERIOD_MS ) != pdPASS) {
		lcd_out("Timer loop start error");
	} else {
		lcd_out("json timer created.");
	}
	//enableCore0WDT();
	//esp_task_wdt_add(NULL);	//enableCore1WDT();

	//loop();
}
