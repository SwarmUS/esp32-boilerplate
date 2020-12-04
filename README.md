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
git clone https://github.com/SwarmUS/esp32-boilerplate
cd esp32-boilerplate
git submodule update --init --recursive
/contrib/esp-idf/install.sh (extension depends on platform, .sh for linux or .bat for windows)
````

## Build
To be able to build for esp-32, you need to have some environment variables source. 
You can run the following command to source the required variables:
````
    source contrib/esp-idf/export.sh
````
After this is done, `echo $IDF_PATH` should output `/path/to/repository/esp32-boilerplate/contrib/esp-idf`.

This repo also contains script to run the cmake with proper arguments. The main can either run locally. 
Either run `./scripts/build.sh`  to build locally or `./scripts/build-esp.sh` to build for esp32. There are also scripts to run on either targets.

## Tests

This projects uses Google Tests for testings, there are some examples of uses.
To run tests you can use 
```make test```

Or use CTest runner (recommended)
```ctest -V```

## Notes

The repo also contains a cmake file to add and run scripts for the esp-idf via cmake and source the variables. 
Attempts were made to integrate it as so, but it proved to be impossible. The files will remain there in case it ever proves feasible.