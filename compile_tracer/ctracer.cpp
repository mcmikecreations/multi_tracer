#ifndef W
#define W 2
#endif
#ifndef H
#define H 2
#endif
#ifndef CC
#define CC 0xffffffff
#endif
#include <array>
#include <cstdint>
#include <cstdio>

using namespace std;
#pragma pack(push)
#pragma pack(1)
struct pixel
{
	union
	{
		uint32_t color = CC;
		struct { uint8_t a, r, g, b; };
	};
};
struct bmp_fileheader
{
	uint16_t signature;
	uint32_t file_size_bytes;
	uint16_t reserved_1;
	uint16_t reserved_2;
	uint32_t data_start_index;
};
struct bmp_infoheader
{
	uint32_t info_size_bytes;
	uint32_t width;
	uint32_t height;
	uint16_t planes;
	uint16_t bits_per_pixel;
	uint32_t compression;
	uint32_t data_padded_size_bytes;
	uint32_t px_per_meter_hor;
	uint32_t px_per_meter_ver;
	uint32_t color_count;
	uint32_t color_count_important;
};
struct bmp_data
{
	pixel data[W*H];
};
struct bmp
{
	bmp_fileheader hfile;
	bmp_infoheader hinfo;
	bmp_data hdata;
};
#pragma pack(pop)
constexpr bmp_fileheader bmp_fill_hfile()
{
	bmp_fileheader r = {};
	r.signature = (uint8_t('M') << 8) + uint8_t('B');
	r.file_size_bytes = sizeof(bmp_fileheader) + sizeof(bmp_infoheader) + sizeof(bmp_data);
	r.reserved_2 = r.reserved_1 = 0;
	r.data_start_index = sizeof(bmp_fileheader) + sizeof(bmp_infoheader);
	return r;
}
constexpr bmp_infoheader bmp_fill_hinfo()
{
	bmp_infoheader r = {};
	r.info_size_bytes = sizeof(bmp_infoheader);
	r.width = W;
	r.height = H;
	r.planes = 1;
	r.bits_per_pixel = sizeof(pixel) * 8;
	r.compression = 0;
	r.data_padded_size_bytes = sizeof(bmp_data);
	r.px_per_meter_hor = 2835; //72 DPI × 39.3701 inches per metre yields 2834.6472
	r.px_per_meter_ver = 2835;
	r.color_count = 0;
	r.color_count_important = 0;
	return r;
}
constexpr bmp_data bmp_fill_hdata()
{
	bmp_data r = {};
	r.data[0].color = 0xFFFF0000;
	r.data[1].color = 0xFF00FF00;
	r.data[2].color = 0xFF0000FF;
	r.data[3].color = 0xFFFFFFFF;
	return r;
}
constexpr bmp bmp_fill()
{
	bmp r = {
		bmp_fill_hfile(),
		bmp_fill_hinfo(),
		bmp_fill_hdata(),
	};
	return r;
}
constexpr std::array<uint8_t,sizeof(bmp)> bmp_dump(bmp val)
{
	std::array<uint8_t,sizeof(bmp)> r = {};
	
	r[0] = val.hfile.signature;
	r[1] = val.hfile.signature >> 8;
	r[2] = val.hfile.file_size_bytes;
	r[3] = val.hfile.file_size_bytes >> 8;
	r[4] = val.hfile.file_size_bytes >> 16;
	r[5] = val.hfile.file_size_bytes >> 24;
	r[6] = val.hfile.reserved_1;
	r[7] = val.hfile.reserved_1 >> 8;
	r[8] = val.hfile.reserved_2;
	r[9] = val.hfile.reserved_2 >> 8;
	r[10] = val.hfile.data_start_index;
	r[11] = val.hfile.data_start_index >> 8;
	r[12] = val.hfile.data_start_index >> 16;
	r[13] = val.hfile.data_start_index >> 24;

	r[14+0] = val.hinfo.info_size_bytes;
	r[14+1] = val.hinfo.info_size_bytes >> 8;
	r[14+2] = val.hinfo.info_size_bytes >> 16;
	r[14+3] = val.hinfo.info_size_bytes >> 24;
	r[14+4] = val.hinfo.width;
	r[14+5] = val.hinfo.width >> 8;
	r[14+6] = val.hinfo.width >> 16;
	r[14+7] = val.hinfo.width >> 24;
	r[14+8] = val.hinfo.height;
	r[14+9] = val.hinfo.height >> 8;
	r[14+10] = val.hinfo.height >> 16;
	r[14+11] = val.hinfo.height >> 24;
	r[14+12] = val.hinfo.planes;
	r[14+13] = val.hinfo.planes >> 8;
	r[14+14] = val.hinfo.bits_per_pixel;
	r[14+15] = val.hinfo.bits_per_pixel >> 8;
	r[14+16] = val.hinfo.compression;
	r[14+17] = val.hinfo.compression >> 8;
	r[14+18] = val.hinfo.compression >> 16;
	r[14+19] = val.hinfo.compression >> 24;
	r[14+20] = val.hinfo.data_padded_size_bytes;
	r[14+21] = val.hinfo.data_padded_size_bytes >> 8;
	r[14+22] = val.hinfo.data_padded_size_bytes >> 16;
	r[14+23] = val.hinfo.data_padded_size_bytes >> 24;
	r[14+24] = val.hinfo.px_per_meter_hor;
	r[14+25] = val.hinfo.px_per_meter_hor >> 8;
	r[14+26] = val.hinfo.px_per_meter_hor >> 16;
	r[14+27] = val.hinfo.px_per_meter_hor >> 24;
	r[14+28] = val.hinfo.px_per_meter_ver;
	r[14+29] = val.hinfo.px_per_meter_ver >> 8;
	r[14+30] = val.hinfo.px_per_meter_ver >> 16;
	r[14+31] = val.hinfo.px_per_meter_ver >> 24;
	r[14+32] = val.hinfo.color_count;
	r[14+33] = val.hinfo.color_count >> 8;
	r[14+34] = val.hinfo.color_count >> 16;
	r[14+35] = val.hinfo.color_count >> 24;
	r[14+36] = val.hinfo.color_count_important;
	r[14+37] = val.hinfo.color_count_important >> 8;
	r[14+38] = val.hinfo.color_count_important >> 16;
	r[14+39] = val.hinfo.color_count_important >> 24;

	for (size_t i = 0; i < val.hinfo.data_padded_size_bytes;i += 4)
	{
		r[14+40+i] = val.hdata.data[i / 4].color;
		r[14+40+i + 1] = val.hdata.data[i / 4].color >> 8;
		r[14+40+i + 2] = val.hdata.data[i / 4].color >> 16;
		r[14+40+i + 3] = val.hdata.data[i / 4].color >> 24;
	}

	return r;
}
int main(int argc, char** argv)
{
	(void)argc;
	(void)argv;
	constexpr auto b = bmp_dump(bmp_fill());
	FILE *f = fopen("/media/sf_proj/git/multi_tracer/out/compile_tracer/test.bmp","wb");
	fwrite(&b,sizeof(b),1,f);
	fclose(f);
	return 0;
}