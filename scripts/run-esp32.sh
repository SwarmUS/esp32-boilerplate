#!/bin/bash
cd build-esp32
python $IDF_PATH/components/esptool_py/esptool/esptool.py -p $1 write_flash @flash_project_args
python $IDF_PATH/tools/idf_monitor.py -p $1 esp32-boilerplate.elf