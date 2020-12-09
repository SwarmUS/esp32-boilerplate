#!/bin/bash
rm -rf build-esp32 && mkdir build-esp32 && cd build-esp32
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/esp-idf/toolchain-esp32.cmake -DTARGET=esp32
cmake --build .