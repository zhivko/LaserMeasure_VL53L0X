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

#include <stdint.h>
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
//#include <Adafruit_SSD1306.h>
#include "SSD1306.h"
//#include <Adafruit_GFX.h>
#include "Adafruit_VL53L0X.h"
#include <vl53l0x_def.h>

#include "debug/lwip_debug.h"
#include "lwip/debug.h"
#include "lwip/stats.h"

#include <esp_heap_caps.h>
#include "esp_heap_trace.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define LED_PIN 16
#define TAG "laser_distance"
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM

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

#include <Preferences.h>
#include "nvs_flash.h"

#include "esp_task_wdt.h"

#if enableTaskManager == 1
	#include "Taskmanager.h"
	static int taskManagerCore = 0;
#endif

//Adafruit_SSD1306 display = Adafruit_SSD1306();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
SSD1306Wire display(0x3c, 5, 4, OLEDDISPLAY_GEOMETRY::GEOMETRY_128_64);

String ssid;
String password;
Preferences preferences;
bool reportingJson = false;
bool shouldSendJson = false;
float timeH;
long previousMs;
long previousJsonSentMs;

const char softAP_ssid[] = "MLIFT";
const char softAP_password[] = "Doitman1";

TaskHandle_t TaskCheckIp;
TaskHandle_t TaskMan;
TaskHandle_t TaskLoop;

WiFiUDP ntpClient;
bool restartNow = false;

AsyncWebServer server(81);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events");
int lcd_y_pos = 0;
char cstr[25];
const char* hostName = "esp32_door";
int jsonReportIntervalMs = 5000;
int jsonFastReportIntervalMs = 500;
int jsonSlowReportIntervalMs = 5000;
uint32_t lastWsClient = -1;
char tempStr[15];
float mm = 0;

void lcd_out(const char*format, ...);
void CheckIpTask(void * parameter);
float WAF_WEIGHT = 0.1;
float weightedAverageFilter(float incomingValue, float previousValue);

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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
String processInput(const char *input) {
	String ret("");
	if (strncmp(input, "wificonnect", 11) == 0) {
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
		lcd_out("wificonnect ssid: %s\n", ssid.c_str());
		esp_restart();
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

void processWsData(const char *data) {
	if (strncmp(data, "ok", 2) != 0) {
		Serial.printf("processWsData: %s\n", data);
		String reply = processInput(data);
		if (reply.length() > 0) {
			ws.textAll(reply.c_str());
		}
	}
}

void wsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
		AwsEventType type, void * arg, uint8_t *data, size_t len) {
	if (type == WS_EVT_CONNECT) {
		lcd_out("ws[%s][%u] [%s] connect\n", server->url(), client->id(),
				client->remoteIP().toString().c_str());

		//client->printf("Hello Client %u :)", client->id());
		delay(100);
		//client->ping();
		jsonReportIntervalMs = jsonFastReportIntervalMs;
		lastWsClient = client->id();

	} else if (type == WS_EVT_DISCONNECT) {
//client disconnected
		lcd_out("%lu ws[%s][%u] [%s] disconnect\n", millis(), server->url(),
				client->id(), client->remoteIP().toString().c_str());
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
	server.addHandler(&events);
	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
	ws.enable(true);

	server.serveStatic("/index.html", SPIFFS, "/index.html", "max-age=600");
	server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico", "max-age=600");
	server.on("/toggleChartsOn", HTTP_GET, [](AsyncWebServerRequest *request) {
		//lcd_out("toggleCharts ON\n");
			shouldSendJson = true;
			jsonReportIntervalMs = jsonFastReportIntervalMs;
			request->send(200, "text/html", "Toggled shouldSendJson ON");
		});
	server.on("/toggleChartsOff", HTTP_GET, [](AsyncWebServerRequest *request) {
		//lcd_out("toggleCharts OFF\n");
			shouldSendJson = false;
			jsonReportIntervalMs = jsonSlowReportIntervalMs;
			request->send(200, "text/html", "Toggled shouldSendJson OFF");
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

	server.on("/target1", HTTP_GET, [](AsyncWebServerRequest *request) {
		int paramsNr = request->params();
		for(int i=0;i<paramsNr;i++) {
			AsyncWebParameter* p = request->getParam(i);
			if(p->name().equalsIgnoreCase("target1"))
			{
				Serial.print("Param name: ");
				Serial.print(p->name());
				Serial.print(" Param value: ");
				Serial.println(p->value());
				String toProcess = String("target1_");
				toProcess.concat("#");
				toProcess.concat(p->value());
				processInput(toProcess.c_str());
			}
		}
		request->send(200, "text/html", "Target1 set OK.");
	});
	server.on("/target2", HTTP_GET, [](AsyncWebServerRequest *request) {
		int paramsNr = request->params();
		for(int i=0;i<paramsNr;i++) {
			AsyncWebParameter* p = request->getParam(i);
			if(p->name().equalsIgnoreCase("target2"))
			{
				Serial.print("Param name: ");
				Serial.print(p->name());
				Serial.print(" Param value: ");
				Serial.println(p->value());
				String toProcess = String("target2_");
				toProcess.concat("#");
				toProcess.concat(p->value().c_str());
				processInput(toProcess.c_str());
			}
		}
		request->send(200, "text/html", "Target1 set OK.");
	});

	server.begin();
	lcd_out("WebServer started.\n");

	MDNS.addService("http", "tcp", 80);
}

void syncTime();

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

	if (len != 0) {
		if (lcd_y_pos > (64 - 8)) {
			lcd_y_pos = 0;
			display.clear();
		}

		display.setFont(ArialMT_Plain_10);
		display.drawString(0, lcd_y_pos, temp);
		lcd_y_pos = lcd_y_pos + 8;
	}

	sprintf(tempStr, "%6.2f", mm);
	display.setFont(ArialMT_Plain_16);
	display.setColor(OLEDDISPLAY_COLOR::WHITE);
	display.drawRect(75, 15, 50, 16);
	display.setColor(OLEDDISPLAY_COLOR::BLACK);
	display.fillRect(75 + 1, 15 + 1, 50 - 2, 16 - 2);
	display.setColor(OLEDDISPLAY_COLOR::WHITE);
	display.drawString(75, 15, tempStr);
	display.display();

//ESP_LOGI(TAG, "%s", temp);
	Serial.printf("%s", temp);
	Serial.flush();

	va_end(arg);
	if (len >= sizeof(loc_buf)) {
		delete[] temp;
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

/*
 void IRAM_ATTR handleIntCapSense(){
 Serial.println ("Capsense interrupt!");
 cap_reading = fdc2212.getReading ();
 }
 */

void blink(int i) {
	for (int j = 0; j < i; j++) {
		digitalWrite(LED_PIN, HIGH);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		digitalWrite(LED_PIN, LOW);
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}

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

void Task1(void * parameter) {
	esp_task_wdt_add(NULL);
	log_i("i2cread task in loop on CORE: %d", xPortGetCoreID());

	for (;;) {

	}
}

void setup() {
	Serial.setDebugOutput(true);
	esp_log_level_set("*", ESP_LOG_VERBOSE);
	esp_log_level_set("I2Cbus", ESP_LOG_WARN);
	esp_log_level_set(TAG, ESP_LOG_VERBOSE);//esp_log_level_set("phy_init", ESP_LOG_INFO);

	Serial.begin(115200);

	byte error, address;
	int nDevices;

	Wire.begin(5, 4, 400000);

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		} else if (error == 4) {
			Serial.print("Unknown error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	display.init(); // initialize with the I2C addr 0x3C (for the 128x64)

	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_10);
	display.setColor(OLEDDISPLAY_COLOR::WHITE);
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.display();

	delay(300);
	Serial.println("Booted display...");
	lcd_out("DoubeLifter START");
	Serial.println("Baud rate: 115200");

	Serial.print("ESP ChipSize:");
	Serial.println(ESP.getFlashChipSize());
	lcd_out("Flash INIT\n");
	if (nvs_flash_init() != ESP_OK) {
		lcd_out("Flash init FAILED!\n");
		nvs_flash_init_partition("nvs");
		nvs_flash_init();
	} else
		lcd_out("Flash init OK.\n");

	Wire.begin();

	if (lox.begin()) {
		lcd_out("VL53L0X not present");
		//while (1)
	} else {
		lcd_out("VL53L0X present");
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

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		lcd_out("Wifi lost connection.\n");
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
			if (ws.hasClient(lastWsClient)) {
				ws.text(lastWsClient, ret.c_str());
			}
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
		//sstartServer();
		}, WiFiEvent_t::SYSTEM_EVENT_GOT_IP6);
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		WiFi.begin();
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

//waitForIp();			//	wifi_mode_t mode = WiFi.getMode();
//	if (mode == WIFI_MODE_AP) {
//		lcd_out("WIFI_MODE_AP");
//
//	}

//	Serial.println("ENA");
	esp_err_t errWdtInit = esp_task_wdt_init(5, false);
	if (errWdtInit != ESP_OK) {
		log_e("Failed to init WDT! Error: %d", errWdtInit);
	}
	disableCore0WDT();
	disableCore1WDT();

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

	WiFi.mode(WIFI_STA);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);
	WiFi.begin(ssid.c_str(), password.c_str());
	WiFi.setSleep(false);

	lcd_out("Starting checkIP task...\n");
	Serial.flush();
	xTaskCreatePinnedToCore(CheckIpTask,			// pvTaskCode
			"checkIpTask",	// pcName
			6096,			// usStackDepth
			NULL,			// pvParameters
			1,			    // uxPriority
			&TaskCheckIp,			// pxCreatedTask
			0);			// xCoreID
	lcd_out("Starting checkIP...Done.\n");

	blink(5);
	lcd_out("Setup Done.\n");
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
uint64_t previousSecondSetter = 0;
long delta;
long start;
void myLoop() {			//ArduinoOTA.handle();

	VL53L0X_RangingMeasurementData_t measureData;
	float previousMm = 0;

//printEncoderInfo();
	for (;;) {

		lox.getSingleRangingMeasurement(&measureData, false);
		if (measureData.RangeStatus != 4) {
			mm = weightedAverageFilter((float) measureData.RangeMilliMeter,
					previousMm);
			lcd_out("");
			previousMm = mm;
		}

		mySecond = esp_timer_get_time() / 1000000.0;
		if (((mySecond % 5 == 0) && (previousSecond != mySecond))
				|| (abs(ESP.getFreeHeap() - previousHeap) > 10000)) {
#ifndef arduinoWebserver
			timeH = (float) (esp_timer_get_time() / (1000000.0 * 60.0 * 60.0));
			Serial.printf("dist [mm]: %6.2f\n", mm);
//			lcd_out(
//					"time[s]: %" PRIu64 " uptime[h]: %.2f core: %d, freeHeap: %u, largest: %u wsLength: %d\n",
//					mySecond, timeH, xPortGetCoreID(), freeheap,
//					heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
//					ws._buffers.length());
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
		}

		//heap_caps_check_integrity_all(true);

		esp_err_t resetOK = esp_task_wdt_reset();
		if (resetOK != ESP_OK) {
			//lcd_out("Failed reset wdt: err %#03x\n", resetOK);
		}

		if ((mySecond % 20 == 0) && (previousSecondSetter != mySecond)) {
			/*
			 Serial.print("Setting setpoint ");
			 if (target1 <= 15000) {
			 target1 = 15300;
			 target2 = 15300;
			 } else {
			 target1 = 15000;
			 target2 = 15000;
			 Serial.printf("%f\n", pid1.getSetpoint());
			 }
			 Serial.printf("%f\n", target1);
			 */
			//heap_caps_print_heap_info(MALLOC_CAP_8BIT);
			//previousSecondSetter = mySecond;
		}

		if (restartNow) {
			//lcd_out("Restarting...\n");
			ESP.restart();
		}

		if ((millis() > (previousJsonSentMs + jsonReportIntervalMs))) {
#if enableTaskManager != 1
			Serial.printf(
					"time[s]: %" PRIu64 " uptime[h]: %.2f core: %d, freeHeap: %u, largest: %u\n",
					mySecond, timeH, xPortGetCoreID(), esp_get_free_heap_size(),
					heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
#endif

			sprintf(cstr, "{\"esp32_heap\":%zu}", esp_get_free_heap_size());
			events.send(cstr, "myevent", millis());
			sprintf(cstr, "{\"uptime_h\":%.2f}", timeH);
			events.send(cstr, "myevent", millis());
			sprintf(cstr, "{\"mm\":%.2f}", mm);
			events.send(cstr, "myevent", millis());

			previousJsonSentMs = millis();
		}
		vTaskDelay(30 / portTICK_PERIOD_MS);
	}
}

void loop() {
	vTaskSuspend(NULL);
	esp_err_t err = esp_task_wdt_reset();
	if (err != ESP_OK) {
		log_e("Failed to feed WDT! Error: %d", err);
	}
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
	ESP_ERROR_CHECK(heap_trace_init_standalone(trace_record, NUM_RECORDS));

	setup();

	lcd_out("Starting LoopTask...");
	Serial.flush();
	xTaskCreatePinnedToCore(Loop,							// pvTaskCode
			"MyLoop",							// pcName
			4000,							// usStackDepth
			NULL,							// pvParameters
			16,							// uxPriority
			&TaskLoop,							// pxCreatedTask
			0);							// xCoreID
	esp_task_wdt_add(TaskLoop);
	lcd_out("Starting LoopTask...Done.\n");
	Serial.flush();

	/*
	 tmr2 = xTimerCreate("MyTimer", pdMS_TO_TICKS(jsonReportIntervalMs),
	 pdTRUE, (void *) id3, &loopCallBack);
	 if ( xTimerStart ( tmr2 , 100 / portTICK_PERIOD_MS ) != pdPASS) {
	 lcd_out("Timer loop start error");
	 } else {
	 lcd_out("json timer created.");
	 }
	 */
//enableCore0WDT();
//esp_task_wdt_add(NULL);	//enableCore1WDT();
//loop();
}

void CheckIpTask(void * parameter) {
	delay(6000);
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

	vTaskDelete(NULL);
}

float weightedAverageFilter(float incomingValue, float previousValue) { // See https://www.tigoe.com/pcomp/code/arduinowiring/37/

	float filteredReading = WAF_WEIGHT * incomingValue
			+ (1.0 - WAF_WEIGHT) * previousValue;
	return filteredReading;

}
