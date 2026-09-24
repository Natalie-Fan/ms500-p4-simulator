cmake_minimum_required(VERSION 3.20)
project(floodsim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(FIRMWARE_DIR "D:/git/NETWORK_P4_FLOOD_DETECTION" CACHE PATH "Firmware repo root")
set(FLOOD_DIR "${FIRMWARE_DIR}/components/as_code/process_flood_detection")
if(NOT EXISTS "${FLOOD_DIR}/flood_capture.h")
  message(FATAL_ERROR
    "Firmware flood sources not found in ${FLOOD_DIR}. "
    "Set -DFIRMWARE_DIR=... (floodsim passes FLOOD_FIRMWARE_DIR).")
endif()
# Recorded so floodsim can tell when the build points at a different firmware checkout.
file(WRITE "${CMAKE_BINARY_DIR}/firmware_dir.txt" "${FIRMWARE_DIR}")

# Firmware sources are compiled in place, never copied. CONFIGURE_DEPENDS
# picks up added/renamed firmware .c files on the next build.
file(GLOB FLOOD_SOURCES CONFIGURE_DEPENDS "${FLOOD_DIR}/*.c")

add_library(flood_objs OBJECT ${FLOOD_SOURCES} sim_shim.c)
target_include_directories(flood_objs PUBLIC ${FLOOD_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(flood_objs PRIVATE -O2 -Wall)
set_target_properties(flood_objs PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(flood_core SHARED $<TARGET_OBJECTS:flood_objs>)
set_target_properties(flood_core PROPERTIES PREFIX "")
target_link_options(flood_core PRIVATE -static-libgcc)

enable_testing()

add_executable(test_shim tests/test_shim.c tests/synth.c)
target_include_directories(test_shim PRIVATE tests)
target_link_libraries(test_shim PRIVATE flood_objs)
add_test(NAME shim COMMAND test_shim)

add_executable(flood_sim flood_sim_main.c sim_io.c)
target_link_libraries(flood_sim PRIVATE flood_objs)
target_compile_options(flood_sim PRIVATE -O2 -Wall)
target_link_options(flood_sim PRIVATE -static)

add_executable(make_fixtures tests/make_fixtures.c tests/synth.c)
target_include_directories(make_fixtures PRIVATE tests)

set(FIX ${CMAKE_BINARY_DIR}/fixtures)
add_test(NAME fixtures COMMAND make_fixtures ${FIX})
set_tests_properties(fixtures PROPERTIES FIXTURES_SETUP sim_fixtures)

function(golden_test name golden)
  string(REPLACE ";" "|" args "${ARGN}")
  add_test(NAME ${name}
           COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:flood_sim> "-DARGS=${args}"
                   -DGOLDEN=${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/${golden}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_golden.cmake)
  set_tests_properties(${name} PROPERTIES FIXTURES_REQUIRED sim_fixtures)
endfunction()

golden_test(golden_device device.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv --mode device)
golden_test(golden_device_geometry device.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv
  --geometry ${FIX}/geometry.json)
golden_test(golden_dense dense.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv --mode dense)

add_test(NAME bad_geometry_rejected
         COMMAND flood_sim --reference ${FIX}/reference.pgm --frames ${FIX}/frames
                 --geometry ${FIX}/bad_geometry.json)
set_tests_properties(bad_geometry_rejected PROPERTIES WILL_FAIL TRUE FIXTURES_REQUIRED sim_fixtures)
