set(MODULE_NAME signal_processing_test)

message("Building " ${MODULE_NAME})

file(GLOB_RECURSE SOURCES *.cpp *.hpp *.h)
add_library(${MODULE_NAME} STATIC ${SOURCES})

set_target_properties(${MODULE_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${MODULE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${MODULE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(${MODULE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)


#target_compile_features(${MODULE_NAME} PUBLIC cxx_std_17)

target_link_libraries(${MODULE_NAME} PUBLIC eigen etl MathUtils SignalProcessors Interpolation)

