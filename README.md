# HiveConnect

Project for the esp32 module and ros simulation node for SwarmUs.

## Requirements

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for building the POSIX target
* [esp-idf](https://github.com/espressif/esp-idf) for the esp32 tooling. v4.2 was chosen for development
* [Clang tools](https://clang.llvm.org/docs/ClangTools.html) are used to match the style and warnings used in the project
    * [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to match the coding style
    * [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) for additional compiler warnings
* [Doxygen](https://github.com/doxygen/doxygen) and [graphviz](https://gitlab.com/graphviz/graphviz/) to generate the documentation

## Setup

This project requires the esp-idf repo cloned and sourced to be able to compile. 
It wil then need to be installed to set the the Extensa toolchain and other stuff. Simply run:
````
git clone -b release/v4.2 --recurse-submodules https://github.com/espressif/esp-idf
esp-idf/install.xx (extension varies from platform to platform, use sh for linux and bat for windows)
git clone https://github.com/SwarmUS/HiveConnect
````

To be able to open a serial port, you will need to add your user to the ``dialout`` group. Run ``sudo adduser YOUR_NAME dialout``.
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
cmake .. -DCMAKE_BUILD_TYPE=Debug <-DCMAKE_TOOLCHAIN_FILE=../cmake/esp-idf/toolchain-esp32.cmake>
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

The project includes a target called ``openocd-flash`` that will flash the program over JTAG with the adafruit ftd2232h breakout board. For other adapters, other .cfg might need to be supplied. 

#### Debugging

The xtensa debugger has a depency on libpython2.7. You can install it using your package manager.

To debug using CLion, you can create a run configuration using the Embedded Gdb Server template. Follow these steps to create this configuration:
1. Create a new configuration using Embedded GDB Server Template
2. Set the executable to the .elf file of the project
3. Select the xtensa gdb located under .espressif (exemple path: ```/home/casto/.espressif/tools/xtensa-esp32-elf/esp-2020r2-8.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gdb```)
4. For download executable, select ``None``. This will be done manually while launching the gdb server (aka openocd)
5. For target remote args, enter ``localhost 3333``
6. For the GDB server, choose the one provided with the esp tool chain, similarly to the gdb (example path: ``/home/casto/.espressif/tools/openocd-esp32/v0.10.0-esp32-20191114/openocd-esp32/bin/openocd``)
7. For the GDB server args, enter these args, modifying the paths to your project: `` -s share/openocd/scripts -f /home/casto/git/Sherbrooke/SwarmUs/HiveConnect/tools/openocd/adafruit-esp.cfg -c "program_esp /home/casto/git/Sherbrooke/SwarmUs/HiveConnect/cmake-build-target/hive_connect.bin 0x10000"``

Debugging is also supported in VS Code. Here is an example launch and task files necessary. Some changes to properly resolve paths are needed (changes to the user and version might be required):
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
            "program": "${workspaceFolder}/cmake-build-target/hive_connect.elf",
            "preLaunchTask": "openocd",
            "setupCommands": [{
                "description": "enable pretty printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            }, {
                "text": "file '${workspaceFolder}/cmake-build-target/hive_connect.elf'"
            }, {
                "text": "target remote :3333"
            }, {
                "text": "monitor program_esp32 ${workspaceFolder}/cmake-build-target/hive_connect.elf 0x10000 verify"
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
            "dependsOn": "update-build",
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
            "command": "bin/openocd -s share/openocd/scripts -f ${workspaceFolder}/tools/openocd/adafruit-esp.cfg -c 'program_esp ${workspaceFolder}/cmake-build-target/hive_connect.bin 0x10000'",
        },
        ,
        {
          "label": "update-build",
          "type": "shell",
          "isBackground": false,
          "options": {
            "cwd": "${workspaceFolder}/cmake-build-target/" // to be changed by user if different build directory
          },
          "command": "make app"
        }
    ]
}
````

IMPORTANT: VS Code has trouble attaching the debugger on the first launch. If this occurs, relaunch the configuration and the program should launch and break on app_main() entry.

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
