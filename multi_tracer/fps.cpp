#include "fps.h"

#include <chrono>
#include <cstdint>
#include <cstring>

// How many frames time values to keep
// The higher the value the smoother the result is...
// Don't make it 0 or less :)
#define FRAME_VALUES 50

using measure_type = std::chrono::system_clock::time_point;
using duration_type = std::chrono::system_clock::duration;

// An array to store frame times:
static duration_type frametimes[FRAME_VALUES];

// Last calculated SDL_GetTicks
static measure_type frametimelast;

// total frames rendered
static std::size_t framecount;

// the value you want
static float framespersecond;

// This function gets called once on startup.
void fps_init() {

    // Set all frame times to 0ms.
    memset(frametimes, 0, sizeof(frametimes));
    framecount = 0;
    framespersecond = 0;
    frametimelast = std::chrono::system_clock::now();

}

void fps_think() {

    std::size_t frametimesindex;
    measure_type getticks;
    std::size_t count;
    std::size_t i;

    // frametimesindex is the position in the array. It ranges from 0 to FRAME_VALUES.
    // This value rotates back to 0 after it hits FRAME_VALUES.
    frametimesindex = framecount % FRAME_VALUES;

    // store the current time
    getticks = std::chrono::system_clock::now();

    // save the frame time value
    frametimes[frametimesindex] = getticks - frametimelast;

    // save the last frame time for the next fpsthink
    frametimelast = getticks;

    // increment the frame count
    framecount++;

    // Work out the current framerate

    // The code below could be moved into another function if you don't need the value every frame.

    // I've included a test to see if the whole array has been written to or not. This will stop
    // strange values on the first few (FRAME_VALUES) frames.
    if (framecount < FRAME_VALUES) {

        count = framecount;

    }
    else {

        count = FRAME_VALUES;

    }

    // add up all the values and divide to get the average frame time.
    duration_type frametime = duration_type{};
    for (i = 0; i < count; i++) {

        frametime += frametimes[i];

    }

    frametime /= count;

    // now to make it an actual frames per second value...
    framespersecond = 1000.f / std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(frametime).count();

}

float fps()
{
    return framespersecond;
}