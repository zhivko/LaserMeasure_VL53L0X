deps_config := \
	/home/klemen/esp/esp-idf-v3.1.1/components/app_trace/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/aws_iot/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/bt/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/driver/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/esp32/Kconfig \
	/home/klemen/esp/DoubleLifter/components/esp32-I2Cbus/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/esp_adc_cal/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/esp_http_client/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/ethernet/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/fatfs/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/freertos/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/heap/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/libsodium/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/log/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/lwip/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/mbedtls/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/nvs_flash/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/openssl/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/pthread/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/spi_flash/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/spiffs/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/tcpip_adapter/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/vfs/Kconfig \
	/home/klemen/esp/esp-idf-v3.1.1/components/wear_levelling/Kconfig \
	/home/klemen/esp/DoubleLifter/components/arduino/Kconfig.projbuild \
	/home/klemen/esp/esp-idf-v3.1.1/components/bootloader/Kconfig.projbuild \
	/home/klemen/esp/esp-idf-v3.1.1/components/esptool_py/Kconfig.projbuild \
	/home/klemen/esp/esp-idf-v3.1.1/components/partition_table/Kconfig.projbuild \
	/home/klemen/esp/esp-idf-v3.1.1/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
