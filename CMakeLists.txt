# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Website_Updater)

set(PARTITION_TABLE_CSV_PATH "${CMAKE_SOURCE_DIR}/partitions.csv")

# Workaround: GCC 14.2.0 on Windows has an internal compiler error (segfault)
# when compiling esp_lcd_panel_rgb.c with -Og. Force -O0 for that file only.
if(TARGET __idf_esp_lcd)
    set_source_files_properties(
        $ENV{IDF_PATH}/components/esp_lcd/rgb/esp_lcd_panel_rgb.c
        TARGET_DIRECTORY __idf_esp_lcd
        PROPERTIES COMPILE_FLAGS "-O0"
    )
endif()
