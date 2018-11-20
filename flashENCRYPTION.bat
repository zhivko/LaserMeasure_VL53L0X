set project_folder=c:\msys32\home\klemen\esp\DoubleLifter

echo %project_folder%

python c:\msys32\home\klemen\esp\esp-idf-v3.1.1\components\esptool_py\esptool\espsecure.py burn_efuse FLASH_CRYPT_CNT
rem python c:\msys32\home\klemen\esp\esp-idf\components\esptool_py\esptool\espsecure.py burn_efuse FLASH_CRYPT_CNT
