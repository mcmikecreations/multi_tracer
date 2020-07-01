#include "scene.h"
#include "random.h"

#include "marl/defer.h"
#include "marl/scheduler.h"
#include "marl/thread.h"
#include "marl/ticket.h"
#include "marl/waitgroup.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#define RENDER_PARALLEL 1

#define PIXELS_PER_JOB 64

void scene::render()
{
	std::size_t size = (std::size_t)_w * (std::size_t)_h;
	memset(_pixels, 0, size * sizeof(float));

#if defined(RENDER_PARALLEL) && RENDER_PARALLEL == 1
	std::size_t job_count = size / PIXELS_PER_JOB + 1;

	marl::WaitGroup render_wait(job_count);

	for (std::size_t j = 0; j < job_count; j++) {
		marl::schedule([=] {
			defer(render_wait.done());

			for (std::size_t i = 0; i < PIXELS_PER_JOB; ++i)
			{
				std::size_t index = j * PIXELS_PER_JOB + i;
				if (index >= size) break;
				_pixels[index * 4 + 0] = random_f32();
				_pixels[index * 4 + 3] = 1.0f;
			}
			});
	}

	render_wait.wait();
#else
	for (std::size_t i = 0; i < size; ++i)
	{
		if (i >= size) break;
		_pixels[i * 4 + 0] = random_f32();
		_pixels[i * 4 + 1] = random_f32();
		_pixels[i * 4 + 2] = random_f32();
		_pixels[i * 4 + 3] = 1.0f;
	}
#endif
}

marl::Scheduler scheduler(marl::Scheduler::Config::allCores());

scene::scene(float width, float height) : _w(width), _h(height), _pixels(new float[(std::size_t)_w * (std::size_t)_h * 4])
{
	scheduler.bind();
}

scene::~scene()
{
	delete[] _pixels;
	defer(scheduler.unbind());
}

float* scene::pixels_get()
{
	return _pixels;
}