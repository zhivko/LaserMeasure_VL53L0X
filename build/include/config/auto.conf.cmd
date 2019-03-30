deps_config := \
	/home/klemen/esp/esp-idf/components/app_trace/Kconfig \
	/home/klemen/esp/esp-idf/components/aws_iot/Kconfig \
	/home/klemen/esp/esp-idf/components/bt/Kconfig \
	/home/klemen/esp/esp-idf/components/driver/Kconfig \
	/home/klemen/esp/esp-idf/components/esp32/Kconfig \
	/home/klemen/esp/DoubleLifter/components/esp32-I2Cbus/Kconfig \
	/home/klemen/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/home/klemen/esp/esp-idf/components/esp_event/Kconfig \
	/home/klemen/esp/esp-idf/components/esp_http_client/Kconfig \
	/home/klemen/esp/esp-idf/components/esp_http_server/Kconfig \
	/home/klemen/esp/esp-idf/components/ethernet/Kconfig \
	/home/klemen/esp/esp-idf/components/fatfs/Kconfig \
	/home/klemen/esp/esp-idf/components/freemodbus/Kconfig \
	/home/klemen/esp/esp-idf/components/freertos/Kconfig \
	/home/klemen/esp/esp-idf/components/heap/Kconfig \
	/home/klemen/esp/esp-idf/components/libsodium/Kconfig \
	/home/klemen/esp/esp-idf/components/log/Kconfig \
	/home/klemen/esp/esp-idf/components/lwip/Kconfig \
	/home/klemen/esp/esp-idf/components/mbedtls/Kconfig \
	/home/klemen/esp/esp-idf/components/mdns/Kconfig \
	/home/klemen/esp/esp-idf/components/mqtt/Kconfig \
	/home/klemen/esp/esp-idf/components/nvs_flash/Kconfig \
	/home/klemen/esp/esp-idf/components/openssl/Kconfig \
	/home/klemen/esp/esp-idf/components/pthread/Kconfig \
	/home/klemen/esp/esp-idf/components/spi_flash/Kconfig \
	/home/klemen/esp/esp-idf/components/spiffs/Kconfig \
	/home/klemen/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/klemen/esp/esp-idf/components/vfs/Kconfig \
	/home/klemen/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/klemen/esp/DoubleLifter/components/arduino/Kconfig.projbuild \
	/home/klemen/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/klemen/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/klemen/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/klemen/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
