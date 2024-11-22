include(FetchContent)

FetchContent_Declare(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG        master  # Use the desired version
)

FetchContent_MakeAvailable(assimp)