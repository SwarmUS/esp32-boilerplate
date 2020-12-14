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

## Using the template
When creating a new repo you can select the Template-Cpp as template instead of an empty repo.

You will need to change the CMakesLists to fit your needs, folders under `src` are libraries, you need to adapt the file `src/CMakeLists.txt` so it reflects your own libraries. You will need to change `tests/src/CMakeLists.txt` too so it reflects your changes. 

## Setup

This project requires the esp-idf repo cloned and sourced to be able to compile. It is included as a submodule and can therefore be initialised by running upon cloning this repo.
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

## Tests

This projects uses Google Tests for testings, there are some examples of uses.
To run tests you can use 
```make test```

Or use CTest runner (recommended)
```ctest -V```
