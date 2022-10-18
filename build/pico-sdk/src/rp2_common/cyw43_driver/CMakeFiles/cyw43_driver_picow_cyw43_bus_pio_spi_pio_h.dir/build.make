# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\Cmake\bin\cmake.exe

# The command to remove a file.
RM = D:\Cmake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Raspberry\DronTx

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Raspberry\DronTx\build

# Utility rule file for cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.

# Include any custom commands dependencies for this target.
include pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\compiler_depend.make

# Include the progress variables for this target.
include pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\progress.make

pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h: pico-sdk\src\rp2_common\cyw43_driver\cyw43_bus_pio_spi.pio.h
	cd D:\Raspberry\DronTx\build\pico-sdk\src\rp2_common\cyw43_driver
	cd D:\Raspberry\DronTx\build

pico-sdk\src\rp2_common\cyw43_driver\cyw43_bus_pio_spi.pio.h: D:\Raspberry\RP2040\pico-sdk\src\rp2_common\cyw43_driver\cyw43_bus_pio_spi.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=D:\Raspberry\DronTx\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating cyw43_bus_pio_spi.pio.h"
	cd D:\Raspberry\DronTx\build\pico-sdk\src\rp2_common\cyw43_driver
	..\..\..\..\pioasm\pioasm.exe -o c-sdk D:/Raspberry/RP2040/pico-sdk/src/rp2_common/cyw43_driver/cyw43_bus_pio_spi.pio D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/cyw43_bus_pio_spi.pio.h
	cd D:\Raspberry\DronTx\build

cyw43_driver_picow_cyw43_bus_pio_spi_pio_h: pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h
cyw43_driver_picow_cyw43_bus_pio_spi_pio_h: pico-sdk\src\rp2_common\cyw43_driver\cyw43_bus_pio_spi.pio.h
cyw43_driver_picow_cyw43_bus_pio_spi_pio_h: pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\build.make
.PHONY : cyw43_driver_picow_cyw43_bus_pio_spi_pio_h

# Rule to build all files generated by this target.
pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\build: cyw43_driver_picow_cyw43_bus_pio_spi_pio_h
.PHONY : pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\build

pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\clean:
	cd D:\Raspberry\DronTx\build\pico-sdk\src\rp2_common\cyw43_driver
	$(CMAKE_COMMAND) -P CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\cmake_clean.cmake
	cd D:\Raspberry\DronTx\build
.PHONY : pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\clean

pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" D:\Raspberry\DronTx D:\Raspberry\RP2040\pico-sdk\src\rp2_common\cyw43_driver D:\Raspberry\DronTx\build D:\Raspberry\DronTx\build\pico-sdk\src\rp2_common\cyw43_driver D:\Raspberry\DronTx\build\pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : pico-sdk\src\rp2_common\cyw43_driver\CMakeFiles\cyw43_driver_picow_cyw43_bus_pio_spi_pio_h.dir\depend

