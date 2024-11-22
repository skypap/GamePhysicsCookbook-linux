include(FetchContent)
FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1 # ou utilisez la dernière version stable de GLM
)
FetchContent_MakeAvailable(glm)
set(GLM_INCLUDE_DIR "${glm_SOURCE_DIR}")

