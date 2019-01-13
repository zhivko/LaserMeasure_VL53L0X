set project_folder="c:\msys32\home\klemen\esp\DoubleLifter"
rem esp32_door.local
echo %project_folder%
python c:/msys32/home/klemen/esp/arduino-esp32/tools/espota.py -i esp32_door.local -p 3232 -f %project_folder%\build\DoubleLifter.bin -d
