#include "encoderSaver.h"

void encoderSaverTask(void * pvParameters) {
	long start;
	long delta;

	while (true) {
		if (rotaryEncoder1.encoderChanged() != 0) {
			//Serial.println("Saving to flash enc1.");
			encoder1_value = rotaryEncoder1.readEncoder();
			start = micros(); // ref: https://github.com/espressif/arduino-esp32/issues/384
			preferences.begin("settings", false);
			preferences.putInt("encoder1_value", rotaryEncoder1.readEncoder());
			preferences.end();
			delta = micros() - start;

			if (delta > 1000) {
				lcd_out("%lu Preferences save completed in %lu us.\n", micros(),
						delta);
			}
		}

		if (rotaryEncoder2.encoderChanged() != 0) {
			//Serial.println("Saving to flash enc2.");
			encoder2_value = rotaryEncoder2.readEncoder();
			start = micros(); // ref: https://github.com/espressif/arduino-esp32/issues/384
			preferences.begin("settings", false);
			preferences.putInt("encoder2_value", rotaryEncoder2.readEncoder());
			preferences.end();
			delta = micros() - start;

			if (delta > 1000) {
				lcd_out("%lu Preferences save completed in %lu us.\n", micros(),
						delta);
			}
		}

		delay(500);
	}
}
