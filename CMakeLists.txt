
cmake_minimum_required(VERSION 3.20)
project(3D_MOVES LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(3D_MOVES src/main.cpp)

target_include_directories(3D_MOVES PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(WIN32)
    target_link_libraries(3D_MOVES PRIVATE
            opengl32
            gdi32
            user32
    )
else()
    message(WARNING "3D_MOVES viewer requires Windows. Build only 3D_MOVES_experiments / 3D_MOVES_tests on Linux.")
    set_target_properties(3D_MOVES PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()

add_executable(3D_MOVES_experiments
        tools/experiments.cpp
)

target_include_directories(3D_MOVES_experiments PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

include(CTest)

if(BUILD_TESTING)
    add_executable(3D_MOVES_tests
            tests/test_core.cpp
    )

    target_include_directories(3D_MOVES_tests PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )

    add_test(NAME core_algorithms COMMAND 3D_MOVES_tests)

    add_executable(3D_MOVES_geometry_tests
            tests/test_geometry_random.cpp
    )

    target_include_directories(3D_MOVES_geometry_tests PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )

    add_test(NAME rotation_collision COMMAND 3D_MOVES_geometry_tests)
endif()