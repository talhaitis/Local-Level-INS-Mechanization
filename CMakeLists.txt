cmake_minimum_required(VERSION 3.10)

# 1. Project Name and Version
project(LocalLevelINSMechanization VERSION 1.0.0 LANGUAGES CXX)

# 2. Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. Compiler Warnings (Optional but recommended)
if (MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

# 4. Add Eigen as an INTERFACE library from third_party/eigen-3.4.0
#    (Eigen is header-only, so no actual library is built)
add_library(Eigen3 INTERFACE)
target_include_directories(Eigen3 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0
)

# 5. Define the executable and include source files
add_executable(ins_mechanization
    src/main.cpp
    src/INSMechanization.cpp
    src/IMUCalibration.cpp
    src/Utilities.cpp 
)

# 6. Include Directories (for your own headers, now in include/)
target_include_directories(ins_mechanization
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 7. Link Libraries (linking the Eigen3 interface library)
target_link_libraries(ins_mechanization PRIVATE Eigen3)

# 8. (Optional) Set the Output Directory for the executable
set_target_properties(ins_mechanization PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
