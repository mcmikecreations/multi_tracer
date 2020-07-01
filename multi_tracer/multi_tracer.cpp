#include "multi_tracer.h"
#include "fps.h"
#include "scene.h"
#include "window.h"

#include <string>

using namespace std;

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

    std::string windowname = "multi_tracer";
	
    const int W = 212, H = 120;
    // Create a screen.
    window wnd(W * 4, H * 4, windowname.c_str());
    // Create scene
    scene scn(W, H);

    fps_init();

    for (bool interrupted = false; !interrupted; )
    {
        // Process events.
        interrupted = !wnd.update();
        // Render scene
        scn.render();
        wnd.transfer(scn.pixels_get(), W, H, 4);

        fps_think();
        wnd.title_set((windowname + " - " + to_string(fps())).c_str());

        //SDL_Delay(1000 / 60);
    }

    return 0;
}
