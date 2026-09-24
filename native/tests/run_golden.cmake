# Usage: cmake -DEXE=<flood_sim> -DARGS=<a|b|c> -DGOLDEN=<file> -P run_golden.cmake
string(REPLACE "|" ";" args "${ARGS}")
execute_process(COMMAND "${EXE}" ${args}
                OUTPUT_VARIABLE out ERROR_VARIABLE err RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "flood_sim exited with ${rc}:\n${err}")
endif()
file(READ "${GOLDEN}" expected)
if(NOT out STREQUAL expected)
  message(FATAL_ERROR "output differs from ${GOLDEN}\n--- got ---\n${out}--- expected ---\n${expected}")
endif()
