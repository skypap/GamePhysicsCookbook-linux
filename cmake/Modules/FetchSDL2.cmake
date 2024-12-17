include(FetchContent)

function(fetch_sdl2)
    FetchContent_Declare(
            SDL2
            GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
            GIT_TAG release-2.28.5
            GIT_SHALLOW TRUE
    )

    # Configuration options pour SDL2
    set(SDL_SHARED TRUE CACHE BOOL "" FORCE)
    set(SDL_STATIC FALSE CACHE BOOL "" FORCE)
    set(SDL_TEST FALSE CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(SDL2)

    # Ajout des variables dans le scope parent
    set(SDL2_FOUND TRUE PARENT_SCOPE)
    set(SDL2_INCLUDE_DIRS ${sdl2_SOURCE_DIR}/include PARENT_SCOPE)
    set(SDL2_LIBRARIES SDL2::SDL2 PARENT_SCOPE)
endfunction()

# Vérifie si SDL2 est déjà trouvé
if(NOT SDL2_FOUND)
    fetch_sdl2()
endif()
