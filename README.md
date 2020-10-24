# Template-Cpp

This is a template repo for C++ for Swarmus.
It's goal is to have a ready project for C/C++ with unit testing and kickstart development.

This is a basic template, you will need more knowledge of CMake as your project grows. You can learn more [here](https://cmake.org/cmake/help/latest/guide/tutorial/index.html) or, for a quick reference, go [here](https://learnxinyminutes.com/docs/cmake/)

## Hierachy

Each folder under src is a library and the main creates an executable.
The libraries are reused for unit testing.
The include folder is where the public headers should be put.

The include, src and test/src should all have a very similar structure

Note that in the provided example, the spi library is dependent on the hal library

````

Template-Cpp/
|-- include/
|   |-- hal             #hal header files
|   |-- spi             #spi header files
|-- src/
|   |-- hal             #hal library files
|   |-- spi             #spi library files
|   `-- main.cpp        #main.cpp, application file
|-- tests/
|   |-- src             #tests location
|       |-- hal             #hal unit tests files
|       |-- spi             #spi unit tests files

````

## Using the template
When creating a new repo you can select the Template-Cpp as template instead of an empty repo.

You will need to change the CMakesLists to fit your needs, folders under `src` are libraries, you need to adapt the file `src/CMakeLists.txt` so it reflects your own libraries. You will need to change `tests/src/CMakeLists.txt` too so it reflects your changes. 

## Build
First clone the repo

```git clone https://github.com/SwarmUS/Template-Cpp.git```

Then you build the project

```
mkdir build
cd build 
cmake ..
make
```

## Tests

This projects uses Google Tests for testings, there are some examples of uses.
To run tests you can use 
```make test```

Or use CTest runner (recommended)
```ctest -V```

## Notes

This repo is not a reference for SwarmUS coding styles.


The test names that were used does not represent our standard, it is just a verbose placeholder for the example.


The flag that treats warnings as error is disabled, you can enable it in `src/CMakeLists.txt`.
