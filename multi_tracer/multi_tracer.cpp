#include "SDL.h"
#include "multi_tracer.h"


using namespace std;

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	
    const int W = 212, H = 120;
    // Create a screen.
    SDL_Window* window = SDL_CreateWindow("Chip8",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W * 4, H * 4, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, W, H);

    // Create scene
    //TODO

    for (bool interrupted = false; !interrupted; )
    {
        // Process events.
        SDL_Event ev;
        while (SDL_PollEvent(&ev))
            switch (ev.type)
            {
            case SDL_QUIT: interrupted = true; break;
            }
        // Render graphics
        Uint32 pixels[W * H] = {};
        // Render scene
        //TODO

        SDL_UpdateTexture(texture, nullptr, pixels, 4 * W);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        SDL_Delay(1000 / 60);
    }

    return 0;
}
