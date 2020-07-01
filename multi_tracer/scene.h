#pragma once

#include "SDL.h"
#include <cstddef>

class scene
{
private:
	float _w, _h;
	float* _pixels;
public:
	scene(float width, float height);
	void render();

	float* pixels_get();

	virtual ~scene();
};