set project_folder="c:\msys32\home\klemen\esp\DoubleLifter"
echo %project_folder%
python c:/msys32/home/klemen/esp/arduino-esp32/tools/espota.py -i 192.168.1.21 -p 3232 -f %project_folder%\build\hello-world.bin -d
