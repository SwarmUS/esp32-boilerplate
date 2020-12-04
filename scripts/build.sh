#!/bin/bash
rm -rf build-local && mkdir build-local && cd build-local
cmake .. 
cmake --build .