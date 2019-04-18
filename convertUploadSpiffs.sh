echo on
cd /home/klemen/esp/DoubleLifter/data_template
pwd
python3.7 -m htmlark -o ../data/index.html index.htm
rm ./index.html
cp ../data/index.html ./index.html
gzip -f ../data/index.html
#rm ../data/index.html

cd /home/klemen/esp/DoubleLifter
rm spiffs.bin
/home/klemen/mkspiffs-0.2.3-arduino-esp32-linux64/mkspiffs -p 256 -b 4096 -s 1433600 -d 5 -c ./data ./spiffs.bin
#/home/klemen/mkspiffs-0.2.3-arduino-esp32-win32/mkspiffs -p 256 -b 4096 -s 1433600 -d 5 -c ./data ./spiffs.bin

python /home/klemen/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyS4 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x291000 ./spiffs.bin


