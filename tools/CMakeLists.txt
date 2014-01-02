find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# Check for g++ C++11 features
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    if(${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "4.6.3")
        message(SEND_ERROR "May not compile because of C++11 features\n g++ version 4.6.3 or greater required!")
    else()
        if(${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "4.7")
            add_definitions(-std=c++0x)
        else()
            add_definitions(-std=c++11)
        endif()
    endif()
endif()

# Build visual application
find_package(Boost 1.44.0 COMPONENTS program_options REQUIRED)

set(EXE_NAME jpsastar-bin)
if(Boost_FOUND)
    add_executable(${EXE_NAME} jpsastar.cpp ../jpsastar/JPSAStar.cpp)
    add_dependencies(${EXE_NAME} ${PROJECT_NAME})
    set_target_properties(${EXE_NAME} PROPERTIES OUTPUT_NAME jpsastar)

    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${EXE_NAME} ${Boost_LIBRARIES})
    target_link_libraries(${EXE_NAME} ${OpenCV_LIBS})
endif()

# Build unit test application
#enable_testing() # needed for add_test() command
find_package(GTest)

set(TEST_NAME unit_tests)
if(GTEST_FOUND)
    add_executable(${TEST_NAME} unit_tests.cpp ../jpsastar/JPSAStar.cpp)
    add_dependencies(${TEST_NAME} ${PROJECT_NAME})

    include_directories(${GTEST_INCLUDE_DIRS})
    target_link_libraries(${TEST_NAME} ${OpenCV_LIBS})
    target_link_libraries(${TEST_NAME} ${GTEST_BOTH_LIBRARIES})
    target_link_libraries(${TEST_NAME} gmock)
endif()
