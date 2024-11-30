cmake_minimum_required(VERSION 3.16)
project(GamePhysics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
include(FetchContent)

# Configurer et fetcher GLM
FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 0.9.9.8  # Tu peux changer la version si tu veux
)
FetchContent_MakeAvailable(glm)
# Dependencies
find_package(SDL2 REQUIRED)
find_package(GLEW REQUIRED)

find_package(OpenGL REQUIRED)

# Fetch ImGui

include(imgui)

## ImGui sources
#add_library(imgui STATIC
#        ${imgui_SOURCE_DIR}/imgui.cpp
#        ${imgui_SOURCE_DIR}/imgui_demo.cpp
#        ${imgui_SOURCE_DIR}/imgui_draw.cpp
#        ${imgui_SOURCE_DIR}/imgui_tables.cpp
#        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
#        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
#        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
#)

# Main project sources
file(GLOB_RECURSE SOURCES
        "Code/*.cpp"
        "Code/*.h"
)

# Include GLAD source
#list(APPEND SOURCES "Code/glad/glad.c")

# But exclude ImGui and main-win32
#list(FILTER SOURCES EXCLUDE REGEX ".*imgui.*")
list(FILTER SOURCES EXCLUDE REGEX ".*main-win32.*")

add_executable(${PROJECT_NAME} ${SOURCES}
        Code/math_config.h)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${SDL2_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}

)

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${SDL2_LIBRARIES}
        imgui
        glm::glm  # Ajouter GL
)
option(USE_DOUBLE_PRECISION "Use double precision for math" OFF)
if(USE_DOUBLE_PRECISION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_DOUBLE_PRECISION)
endif()
# Copy assets
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/Assets $<TARGET_FILE_DIR:${PROJECT_NAME}>/Assets
)