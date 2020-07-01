#include "window.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>

#define WINDOW_BACKEND_SDL 2
#define WINDOW_BACKEND_OLC 3
#define WINDOW_BACKEND WINDOW_BACKEND_OLC

#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
#include "SDL.h"
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#endif

enum class _pointers : int
{
#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	wnd = 0,
	renderer = 1,
	texture = 2,
	pixels = 3,
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	wnd = 0,
	thread = 1,
	texture = 2,
	pixels = 3,
#endif
};

#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
std::mutex mtx;
std::atomic_bool atm = true;

class WindowPGE : public olc::PixelGameEngine
{
private:
	olc::Sprite* _surface;
public:
	WindowPGE(olc::Sprite* surface, const char* title) : _surface(surface)
	{
		sAppName = title;
	}
	void title_set(const char* title) { sAppName = title; }
public:
	bool OnUserCreate() override
	{
		atm = true;
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		mtx.lock();
		this->DrawSprite(0, 0, _surface);
		mtx.unlock();
		return true;
	}

	bool OnUserDestroy() override
	{
		atm = false;
		return true;
	}
};
#endif

window::window(int width, int height, const char* title)
	: _w(width), _h(height),
	_surf_w(width / 4), _surf_h(height / 4),
	_title(title)
{
#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	SDL_Window* window = SDL_CreateWindow(title,
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_RESIZABLE);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
	SDL_Texture* texture = SDL_CreateTexture(renderer,
		SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, _surf_w, _surf_h);

	_ptr[(int)_pointers::wnd] = window;
	_ptr[(int)_pointers::renderer] = renderer;
	_ptr[(int)_pointers::texture] = texture;
	_ptr[(int)_pointers::pixels] = new std::uint32_t[(std::size_t)_surf_w * (std::size_t)_surf_h];
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	olc::Sprite* texture = new olc::Sprite { _surf_w, _surf_h };
	WindowPGE* window = new WindowPGE { texture, title };
	window->title_set(title);
	window->Construct(_surf_w, _surf_h, _w / _surf_w, _h / _surf_h);
	std::thread* t = new std::thread(&WindowPGE::Start, window);
	_ptr[(int)_pointers::wnd] = window;
	_ptr[(int)_pointers::thread] = t;
	_ptr[(int)_pointers::texture] = texture;
	_ptr[(int)_pointers::pixels] = texture->GetData();
#endif
}

window::~window()
{
#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	SDL_DestroyTexture(static_cast<SDL_Texture*>(_ptr[(int)_pointers::texture]));
	SDL_DestroyRenderer(static_cast<SDL_Renderer*>(_ptr[(int)_pointers::renderer]));
	SDL_DestroyWindow(static_cast<SDL_Window*>(_ptr[(int)_pointers::wnd]));
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	std::thread* t = static_cast<std::thread*>(_ptr[(int)_pointers::thread]);
	t->join();
	delete t;
	delete _ptr[(int)_pointers::wnd];
	delete _ptr[(int)_pointers::texture];
#endif
}

void window::title_set(const char* title)
{
	_title = title;
#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	SDL_SetWindowTitle(static_cast<SDL_Window*>(_ptr[(int)_pointers::wnd]), title);
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	static_cast<WindowPGE*>(_ptr[(int)_pointers::wnd])->title_set(title);
#endif
}

std::string window::title_get()
{
	return _title;
}

void window::transfer(float* source, float width, float height, int channels)
{
	std::size_t w = (std::size_t)width;
	std::size_t h = (std::size_t)height;

	if (source == nullptr) return;
	if (w != _surf_w || h != _surf_h) return;
	if (channels != 4) return;

#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	Uint32* pixels = static_cast<Uint32*>(_ptr[(int)_pointers::pixels]);

	for (std::size_t j = 0; j < h; ++j)
	{
		for (std::size_t i = 0; i < w; ++i)
		{
			std::uint8_t r, g, b, a;
			std::size_t index_buffer = j * w + i;
			std::size_t index_pixels = index_buffer * 4;
			r = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 0] * 256), 0, 255));
			g = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 1] * 256), 0, 255));
			b = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 2] * 256), 0, 255));
			a = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 3] * 256), 0, 255));
			pixels[index_buffer] = (r << 24) + (g << 16) + (b << 8) + (a);
		}
	}

	SDL_UpdateTexture(
		static_cast<SDL_Texture*>(_ptr[(int)_pointers::texture]), 
		nullptr, 
		_ptr[(int)_pointers::pixels], 
		4 * _surf_w
	);
	SDL_RenderCopy(
		static_cast<SDL_Renderer*>(_ptr[(int)_pointers::renderer]), 
		static_cast<SDL_Texture*>(_ptr[(int)_pointers::texture]), 
		nullptr, nullptr);
	SDL_RenderPresent(static_cast<SDL_Renderer*>(_ptr[(int)_pointers::renderer]));
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	olc::Pixel* pixels = static_cast<olc::Pixel*>(_ptr[(int)_pointers::pixels]);
	mtx.lock();
	for (std::size_t j = 0; j < h; ++j)
	{
		for (std::size_t i = 0; i < w; ++i)
		{
			std::uint8_t r, g, b, a;
			std::size_t index_buffer = j * w + i;
			std::size_t index_pixels = index_buffer * 4;
			r = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 0] * 256), 0, 255));
			g = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 1] * 256), 0, 255));
			b = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 2] * 256), 0, 255));
			a = static_cast<std::uint8_t>(std::clamp<std::uint16_t>(static_cast<std::uint16_t>(source[index_pixels + 3] * 256), 0, 255));
			pixels[index_buffer].n = (r << 24) + (g << 16) + (b << 8) + (a);
		}
	}
	mtx.unlock();
#endif
}

bool window::update()
{
#if defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_SDL
	SDL_Event ev;
	while (SDL_PollEvent(&ev))
		switch (ev.type)
		{
		case SDL_QUIT: return false;
		case SDL_KEYDOWN:
			/* Check the SDLKey values and move change the coords */
			switch (ev.key.keysym.sym) {
			case SDLK_ESCAPE:
				return false;
			default:
				break;
			}
		}
	return true;
#elif defined (WINDOW_BACKEND) && WINDOW_BACKEND == WINDOW_BACKEND_OLC
	WindowPGE* window = static_cast<WindowPGE*>(_ptr[(int)_pointers::wnd]);
	if (!atm.load()) return false;
	if (window->GetKey(olc::Key::ESCAPE).bPressed) return false;
	return true;
#endif
}