set project_folder=c:\msys32\home\klemen\esp\DoubleLifter

echo %project_folder%

python c:\msys32\home\klemen\esp\esp-idf-v3.1.1\components\esptool_py\esptool\esptool.py --chip esp32 --port COM24 --baud 921600 --before "default_reset" --after "hard_reset" write_flash --compress -z --flash_mode "dio" --flash_freq "40m" --flash_size detect 0x1000 "%project_folder%\build\bootloader\bootloader.bin" 0x10000 "%project_folder%\build\hello-world.bin" 0x8000 "%project_folder%\build\partitions.bin"
rem 0x291000 "%project_folder%\spiffs.bin"
