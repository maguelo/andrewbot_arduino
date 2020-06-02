
set(BINARY ${CMAKE_PROJECT_NAME}_test)

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

set(SOURCES ${TEST_SOURCES})

add_executable(${BINARY} ${TEST_SOURCES})

add_test(NAME ${BINARY} COMMAND ${BINARY})


target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}_lib gtest)


# #add_subdirectory(arduino-mock)

# include_directories(
#     ${ARDUINO_MOCK_INCLUDE_DIRS}
#     ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest/include
#     ${ARDUINO_MOCK_LIBS_DIR}/lib/gmock/gmock/src/gmock/include
# )

# #file(GLOB SRC "DigitalReadSerial_unittest.cc")
# #add_executable(arduino_mock_sample DigitalReadSerial_unittest.cc)
# add_executable(${BINARY} ${TEST_SOURCES})

# #target_link_libraries(arduino_mock_sample
# target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}_lib #gtest
#     ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest-build/libgtest.a
#     ${ARDUINO_MOCK_LIBS_DIR}/lib/gmock/gmock/src/gmock-build/libgmock.a
#     ${ARDUINO_MOCK_LIBS_DIR}/dist/lib/libarduino_mock.a
#     ${CMAKE_THREAD_LIBS_INIT}
# )

# #add_dependencies(arduino_mock_sample arduino_mock)
# add_dependencies(${BINARY} arduino_mock)

# enable_testing()
# #add_test(arduino_mock_sample_1 arduino_mock_sample)
# add_test(NAME ${BINARY} COMMAND ${BINARY})