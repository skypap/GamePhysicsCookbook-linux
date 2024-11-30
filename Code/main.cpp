#include <GL/glew.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include "IWindow.h"
// #include "matrices.h"
#include "imgui.h"
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>

// Debug memory leaks
#if _DEBUG
    #include <stdlib.h>
    #include <crtdbg.h>
    unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);
#endif

double GetMilliseconds() {
    return SDL_GetTicks64();
}

int main(int argc, char* argv[]) {
    IWindow* pWindowInstance = IWindow::GetInstance();
    
    if (pWindowInstance->GetQuitFlag()) {
        SDL_Log("Something went wrong, could not initialize!\n");
        return 1;
    }

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
        return 1;
    }

    // GL 3.3 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);;
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    // Create window
    SDL_Window* window = SDL_CreateWindow(
        pWindowInstance->GetTitle(),
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        pWindowInstance->GetWidth(), pWindowInstance->GetHeight(),
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Create OpenGL context
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) {
        SDL_Log("Failed to create GL context: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    // Après la création du contexte, ajoutons :
    if (glewInit() != GLEW_OK) {
        SDL_Log("Failed to initialize GLEW");
        return 1;
    }

    // Initialisation OpenGL de base
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // Dans la boucle principale, avant le rendu :
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Initialize the window
    pWindowInstance->OnInitialize();
    pWindowInstance->MarkAsShown();

    // Main loop variables
    bool running = true;
    Uint32 next_game_tick = SDL_GetTicks();
    double lastTime = GetMilliseconds();
    double fixed_millis = pWindowInstance->GetFixedFPS() / 1000.0;
    double fixed_elapsed = 0.0;

    // Main loop
    while (!pWindowInstance->GetQuitFlag() && running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_WINDOWEVENT && 
                     event.window.event == SDL_WINDOWEVENT_RESIZED) {
                int width = event.window.data1;
                int height = event.window.data2;
                pWindowInstance->Resize(width, height);
                if (pWindowInstance->WasWindowShown()) {
                    pWindowInstance->OnResize(width, height);
                }
            }
            else if (event.type == SDL_MOUSEMOTION && !io.WantCaptureMouse) {
                pWindowInstance->OnMouseMove(event.motion.x, event.motion.y);
            }
            else if ((event.type == SDL_MOUSEBUTTONDOWN || 
                     event.type == SDL_MOUSEBUTTONUP) && !io.WantCaptureMouse) {
                int button;;
                switch (event.button.button) {
                    case SDL_BUTTON_LEFT: button = MOUSE_LEFT; break;
                    case SDL_BUTTON_RIGHT: button = MOUSE_RIGHT; break;
                    case SDL_BUTTON_MIDDLE: button = MOUSE_MIDDLE; break;
                }
                if (event.type == SDL_MOUSEBUTTONDOWN) {
                    pWindowInstance->OnMouseDown(button);
                } else {
                    pWindowInstance->OnMouseUp(button);
                }
            }
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Update title if needed
        if (pWindowInstance->GetAndResetTitleDirtyFlag()) {
            SDL_SetWindowTitle(window, pWindowInstance->GetTitle());
        }

        // Handle fullscreen toggle
        static bool fullscreen = false;
        if (fullscreen != pWindowInstance->GetFullScreen()) {
            fullscreen = pWindowInstance->GetFullScreen();
            SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
        }

        // Update timing
        double time = GetMilliseconds();
        float deltaTime = float(time - lastTime) * 0.001f;
        lastTime = time;

        // Update and render
        pWindowInstance->OnUpdate(deltaTime);

        fixed_elapsed += deltaTime;
        while (fixed_elapsed > fixed_millis) {
            pWindowInstance->OnFixedUpdate(fixed_millis);
            fixed_elapsed -= fixed_millis;
        }

        pWindowInstance->OnRender();

        // Render ImGui
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        SDL_GL_SwapWindow(window);

        // FPS limiting
        int SKIP_TICKS = 1000 / pWindowInstance->GetTargetFPS();
        Uint32 current_tick = SDL_GetTicks();
        if (next_game_tick > current_tick) {
            SDL_Delay(next_game_tick - current_tick);
        }
        next_game_tick += SKIP_TICKS;
    }

    // Cleanup
    pWindowInstance->OnShutdown();
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    CleanupMemory(pWindowInstance);

    #if _DEBUG
        _CrtDumpMemoryLeaks();
    #endif

    return 0;
}