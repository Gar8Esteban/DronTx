# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Raspberry/RP2040/pico-sdk/tools/pioasm"
  "D:/Raspberry/DronTx/build/pioasm"
  "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm"
  "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/tmp"
  "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src"
  "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Raspberry/DronTx/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
