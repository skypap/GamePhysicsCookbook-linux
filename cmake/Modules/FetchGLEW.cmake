include(FetchContent)

function(fetch_glew)
    FetchContent_Declare(
            glew
            GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
            GIT_TAG glew-cmake-2.2.0
            GIT_SHALLOW TRUE
    )

    set(glew-cmake_BUILD_SHARED ON CACHE BOOL "" FORCE)
    set(glew-cmake_BUILD_STATIC OFF CACHE BOOL "" FORCE)
    set(BUILD_UTILS OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(glew)

    # Définir les variables pour le scope parent
    set(GLEW_FOUND TRUE PARENT_SCOPE)
    set(GLEW_INCLUDE_DIRS ${glew_SOURCE_DIR}/include PARENT_SCOPE)
    set(GLEW_LIBRARIES libglew_shared PARENT_SCOPE)
endfunction()

# Vérifie si GLEW est déjà trouvé
if(NOT GLEW_FOUND)
    fetch_glew()
endif()
