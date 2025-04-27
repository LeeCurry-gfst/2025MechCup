# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "E:/EspressifIDE/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/tmp"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/src/bootloader-stamp"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/src"
  "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/scut/2024-2025/Competitions/MechCup/ESP32/ESPBLEHC-08ControllerNEW/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
