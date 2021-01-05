# Esp-32 boilerplate

This is a boilerplate repo for esp-32 code for SwamUs.
It's goal is to have a ready project for esp32 with unit testing and kickstart development and integration with the esp-idf build tools and library.

It provides a way to run a simple loop that shows hardware information and prints an Hello World message.
It can either be run locally or on an esp-32 module.


## Hierachy

Each folder under src is a library referring to an esp-idf component. It offers stubs to run built in components locally or links to the idf-components depending on the cmake arguments.
As code grows and the architecture becomes clearer, this is subject to change. 

The stub libraries are reused for unit testing with GTest.

````
esp32-boilerplate/
|-- src/
|   |-- esp32/
|   |   |-- stub            #stub library files
|   |   |   |-- include     #library header files. Should match include path of the esp-idf component
|   |   |   |-- src         #library source files
|   |-- freertos/
|   |   |-- stub            #stub library files
|   |   |   |-- include     #library header files. Should match include path of the esp-idf component
|   |   |   |-- src         #library source files
|   |-- spi_flash/
|   |   |-- stub            #stub library files
|   |   |   |-- include     #library header files. Should match include path of the esp-idf component
|   |   |   |-- src         #library source files
|   -- main.cpp        #main.c, app_main for esp-idf. Hook for main function contained in esp32 stub
|-- tests/
|   |--esp32/              #directory for esp32 stub tests
````

## Setup

This project requires the esp-idf repo cloned and sourced to be able to compile. 
It wil then need to be installed to set the the Extensa toolchain and other stuff. Simply run:
````
git clone -b release/v4.2 --recurse-submodules https://github.com/espressif/esp-idf
esp-idf/install.xx (extension varies from platform to platform, use sh for linux and bat for windows)
git clone https://github.com/SwarmUS/esp32-boilerplate
````

## Build
To be able to build for esp-32, you need to have some environment variables source. 
You can run the following command to source the required variables:
````
    source path/to/esp-idf/export.xx (extension varies from platform to platform, use sh for linux and bat for windows)
````
After this is done, `echo $IDF_PATH` should output `/path/to/esp-idf`.

To build for esp or host, do the following commands. Supplying the to cmake will build for esp32.
````
mkdir build
cd build
cmake .. <-DCMAKE_TOOLCHAIN_FILE=../cmake/esp-idf/toolchain-esp32.cmake -DTARGET=esp32>
make
````


For CLion users, since you need to source a script before running the cmake from clion, you just need to source the script before launching clion from a terminal.
Simply run:
````
    source path/to/esp-idf/export.xx (extension varies from platform to platform, use sh for linux and bat for windows)
    clion
````
Then, in ``File->Settings->Build,Execution,Deployment->CMake``, add a profile, set the type to debug and copy the following arguments in CMake Options: ``-DCMAKE_TOOLCHAIN_FILE=cmake/esp-idf/toolchain-esp32.cmake -DTARGET=esp32``.
This has been tested on Ubuntu 20.04 but the same logic should apply to any system.

## Running
As can be seen in the scripts, the target can be run locally with a make run command and on the esp32 using the idf.py python script to flash and open a serial on the esp32.
The esp32 target could be changed in the future to use openocd.

The project includes a target called ``openocd-flash`` that will flash the program over JTAG with the adafruit ftd2232h breakout board. For other adapters, other .cfg might need to be supplied. Debugging is supported in VS Code. Here is the launch and task files necessary. Some changes to properly resolve paths are needed:
launch.json
````json
    {
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "OpenOCD Debug",
            "type": "cppdbg",
            "request": "launch",
            "miDebuggerPath": "/home/casto/.espressif/tools/xtensa-esp32-elf/esp-2020r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gdb", // to be changed by user
            "program": "${workspaceFolder}/cmake-build-target/esp32-boilerplate.elf",
            "preLaunchTask": "openocd",
            "setupCommands": [{
                "description": "enable pretty printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            }, {
                "text": "file '${workspaceFolder}/cmake-build-target/esp32-boilerplate.elf'"
            }, {
                "text": "target remote :3333"
            }, {
                "text": "monitor program_esp32 ${workspaceFolder}/cmake-build-target/esp32-boilerplate.elf 0x10000 verify"
            }, {
                "text": "monitor reset halt"
            }, {
                "text": "thb app_main"
            }],
            "cwd": "${workspaceFolder}",
            "externalConsole": false
        }
    ]
}
````
tasks.json
````json
    {
    "version": "2.0.0",
    "tasks": [
        {
            "label": "openocd",
            "type": "shell",
            "isBackground": true,
            "problemMatcher": [
                {
                  "pattern": [
                    {
                      "regexp": ".",
                      "file": 1,
                      "location": 2,
                      "message": 3
                    }
                  ],
                  "background": {
                    "activeOnStart": true,
                    "beginsPattern": ".",
                    "endsPattern": ".",
                  }
                }
              ],
            "options": 
            {
                "cwd": "/home/casto/.espressif/tools/openocd-esp32/v0.10.0-esp32-20191114/openocd-esp32" // to be changed by user
            },
            "command": "bin/openocd -s share/openocd/scripts -f ${workspaceFolder}/tools/openocd/adafruit-esp.cfg -c 'program_esp ${workspaceFolder}/cmake-build-target/esp32-boilerplate.bin 0x10000'",
        }
    ]
}
````

IMPORTANT: For some reason, VS Code has trouble attaching the debugger on the first launch. If this occurs, relaunch the configuration and the program should launch and break on app_main() entry.

## Connections to ftd2232h breakout board

````
Connections:
FTD2232H        ESP32       Signal
D0              D13         TCLK
D1              D12         TDI
D2              D15         TMS
D3              D14         TDO
GND             GND         GND
````

## Tests

This projects uses Google Tests for testings, there are some examples of uses.
To run tests you can use 
```make test```

Or use CTest runner (recommended)
```ctest -V```
