#pragma once

#include <map>
#include <string>

class window
{
private:
	std::map<int, void*> _ptr;
	int _w, _h;
	int _surf_w, _surf_h;
	std::string _title;
public:
	window(int width, int height, const char* title);
	virtual ~window();

	void title_set(const char* title);
	std::string title_get();

	bool update();
	void transfer(float* source, float width, float height, int channels);
};