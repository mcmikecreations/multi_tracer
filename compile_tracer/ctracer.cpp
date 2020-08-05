#ifndef W
#define W 640
#endif
#ifndef H
#define H 480
#endif
#ifndef CC
#define CC 0xffffffff
#endif
#ifndef M_E
#define M_E        2.71828182845904523536   // e
#endif
#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <tuple>
#include <type_traits>
using namespace std;
constexpr inline uint8_t cmf(float value) {
	if (value != value) return 255;
	if (value >= 255.0f) return 255;
	if (value <= 0.0f) return 0;
	return (uint8_t)value;
}
struct pixel
{
	uint8_t a, r, g, b;
	constexpr pixel() : a(CC >> 24), r((CC >> 16) & 0xff), g((CC >> 8) & 0xff), b(CC & 0xff) {}
	constexpr pixel(uint8_t _a, uint8_t _r, uint8_t _g, uint8_t _b) : a(_a), r(_r), g(_g), b(_b) {}
	constexpr pixel(float _a, float _r, float _g, float _b) : 
		a(cmf(_a * 255)), 
		r(cmf(_r * 255)), 
		g(cmf(_g * 255)), 
		b(cmf(_b * 255))
		{}
	constexpr pixel(uint8_t _r, uint8_t _g, uint8_t _b) : a(255), r(_r), g(_g), b(_b) {}
	constexpr pixel(float _r, float _g, float _b) : 
		a(255), 
		r(cmf(_r * 255)), 
		g(cmf(_g * 255)), 
		b(cmf(_b * 255))
		{}
};
#pragma region powf
namespace details
{
    using ifloat = long double;
    constexpr float powf(float base, float term);
    constexpr float powf_integral(float base, int term);
    constexpr float logf(float base);
    constexpr float expf(float term);

    constexpr float sinf(float x);
    constexpr float cosf(float x);
    constexpr float tanf(float x);

    constexpr float fabs(float x);
    constexpr float sqrtf(float x);
} // namespace details

constexpr float details::sqrtf(float x)
{
    ifloat l = 0, m = 0, r = x;
    do
    {
        m = (l + r) / 2;
        if (m * m < x)
        {
            l = m;
        }
        else
        {
            r = m;
        }
    } while (fabs(m * m - x) > std::numeric_limits<ifloat>::epsilon());
    return m;
}

constexpr float details::fabs(float x)
{
    return x < 0.0f ? -x : x;
}

constexpr float details::tanf(float x)
{
    float sin = sinf(x);
    float cos = cosf(x);
    if (cos == 0) return std::numeric_limits<float>::infinity();
    return sin / cos;
}

constexpr float details::sinf(float x)
{
    ifloat sum = x;
    ifloat addition = x;
    for (int n = 1; n < 50000; ++n)
    {
        addition = -addition * (x * x / (ifloat)(2 * n * (2 * n + 1)));
        sum += addition;
        if (fabs(addition) < std::numeric_limits<ifloat>::epsilon()) break;
    }
    return (float)sum;
}

constexpr float details::cosf(float x)
{
    ifloat sum = 1;
    ifloat addition = 1;
    for (int n = 1; n < 50000; ++n)
    {
        addition = -addition * (x * x / (ifloat)(2 * n * (2 * n - 1)));
        sum += addition;
        if (fabs(addition) < std::numeric_limits<ifloat>::epsilon()) break;
    }
    return (float)sum;
}

constexpr float details::powf(float base, float term)
{
    if (term == 0.0f) return 1.0f;
    if (base == 0.0f) return 0.0f;
    float result = 0.0f;
    if (term > 10.0f)
    {
        result = powf_integral(base, (int)term) * expf((term - (float)((int)term)) * logf(base));
    }
    else
    {
        result = expf(term * logf(base));
    }
    if (
        (base > 0.0f && base < 1.0f && term > 0.0f && result > 1.0f)
        ||
        (base > 0.0f && result < 0.0f)
        )
    {
        // Expression failed to converge, but we can re-calculate it approximately
        return powf_integral(base, (int)term);
    }
    return result;
}

constexpr float details::powf_integral(float base, int term)
{
    ifloat result = 1.0f;
    if (term > 0)
    {
        for (int i = 1; i <= term; ++i)
        {
            result *= base;
        }
    }
    if (term < 0)
    {
        for (int i = term; i < 0; ++i)
        {
            result /= base;
        }
    }
    return result;
}

constexpr float details::logf(float base)
{
    if (base <= -1.0f) return std::numeric_limits<float>::signaling_NaN();
    if (base > 1.0f) return std::numeric_limits<float>::signaling_NaN();
    if (base == 0.0f) return -std::numeric_limits<float>::infinity();

    ifloat x = base - 1.0f;
    ifloat sum = 0.0f;

    ifloat curX = x;
    ifloat curNeg = 1.0f;
    ifloat addition = 0.0f;
    for (int n = 1; n < 50000; ++n)
    {
        addition = curNeg * curX / (ifloat)n;
        sum += addition;
        if (fabs(addition) < std::numeric_limits<ifloat>::epsilon()) break;
        curNeg *= -1.0f;
        curX *= x;
    }
    return (float)sum;
}

constexpr float details::expf(float term)
{
    if (term == 0.0f) return 1.0f;
    if (term == 1.0f) return M_E;
    ifloat curX = term;
    ifloat curFact = 1.0f;
    ifloat sum = 1.0f;
    ifloat addition = 0.0f;
    for (int i = 50000 - 1; i > 0; --i)
    {
        addition = term / (ifloat)i;
        sum = 1.0f + addition * sum;
        if (fabs(addition) < std::numeric_limits<ifloat>::epsilon()) break;
    }
    if (sum == std::numeric_limits<ifloat>::infinity() || sum == -std::numeric_limits<ifloat>::infinity()) return 0.0f;
    return (float)sum;
}
#pragma endregion
#pragma region Raytracer math
// https://github.com/ssloy/tinyraytracer
//------------------------------------------------------------------------------
template <size_t DIM, typename T> struct vec {
    vec() { for (size_t i=DIM; i--; data_[i] = T()); }
    constexpr       T& operator[](const size_t i)       { return data_[i]; }
    constexpr const T& operator[](const size_t i) const { return data_[i]; }
private:
    T data_[DIM];
};

typedef vec<2, float> Vec2f;
typedef vec<3, float> Vec3f;
typedef vec<3, int  > Vec3i;
typedef vec<4, float> Vec4f;

template <typename T> struct vec<2,T> {
    constexpr vec() : x(T()), y(T()) {}
    constexpr vec(T X, T Y) : x(X), y(Y) {}
    template <class U> vec<2,T>(const vec<2,U> &v);
    constexpr       T& operator[](const size_t i)       { return i<=0 ? x : y; }
    constexpr const T& operator[](const size_t i) const { return i<=0 ? x : y; }
    T x,y;
};

template <typename T> struct vec<3,T> {
    constexpr vec() : x(T()), y(T()), z(T()) {}
    constexpr vec(T X, T Y, T Z) : x(X), y(Y), z(Z) {}
    constexpr       T& operator[](const size_t i)       { return i<=0 ? x : (1==i ? y : z); }
    constexpr const T& operator[](const size_t i) const { return i<=0 ? x : (1==i ? y : z); }
    constexpr float norm() { return details::sqrtf(x*x+y*y+z*z); }
    constexpr vec<3,T> & normalize(T l=1)
	{ 
		*this = (*this)*(l/norm());
		return *this; 
	}
	constexpr operator pixel()
	{
		return pixel(x, y, z);
	}
    T x,y,z;
};

template <typename T> struct vec<4,T> {
    constexpr vec() : x(T()), y(T()), z(T()), w(T()) {}
    constexpr vec(T X, T Y, T Z, T _W) : x(X), y(Y), z(Z), w(_W) {}
    constexpr       T& operator[](const size_t i)       { return i<=0 ? x : (1==i ? y : (2==i ? z : w)); }
    constexpr const T& operator[](const size_t i) const { return i<=0 ? x : (1==i ? y : (2==i ? z : w)); }
	constexpr operator pixel()
	{
		return pixel(w, x, y, z);
	}
    T x,y,z,w;
};

template<size_t DIM,typename T> T constexpr operator*(const vec<DIM,T>& lhs, const vec<DIM,T>& rhs) {
    T ret = T();
    for (size_t i=DIM; i--; ret+=lhs[i]*rhs[i]);
    return ret;
}

template<size_t DIM,typename T> constexpr vec<DIM,T> operator+(vec<DIM,T> lhs, const vec<DIM,T>& rhs) {
    for (size_t i=DIM; i--; lhs[i]+=rhs[i]);
    return lhs;
}

template<size_t DIM,typename T> constexpr vec<DIM,T> operator-(vec<DIM,T> lhs, const vec<DIM,T>& rhs) {
    for (size_t i=DIM; i--; lhs[i]-=rhs[i]);
    return lhs;
}

template<size_t DIM,typename T,typename U> constexpr vec<DIM,T> operator*(const vec<DIM,T> &lhs, const U& rhs) {
    vec<DIM,T> ret;
    for (size_t i=DIM; i--; ret[i]=lhs[i]*rhs);
    return ret;
}

template<size_t DIM,typename T> constexpr vec<DIM,T> operator-(const vec<DIM,T> &lhs) {
    return lhs*T(-1);
}

template <typename T> constexpr vec<3,T> cross(vec<3,T> v1, vec<3,T> v2) {
    return vec<3,T>(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}

//------------------------------------------------------------------------------

constexpr Vec3f to_vec3f(const pixel& from)
{
	return Vec3f(float(from.r) / 255.0f, float(from.g) / 255.0f, float(from.b) / 255.0f);
}
#pragma endregion
#pragma region Raytracer data
struct Light {
    constexpr Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    constexpr Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    constexpr Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    constexpr Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    constexpr bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = details::sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};
#pragma endregion
template <size_t SphereCount = 4, size_t LightCount = 3>
constexpr 
std::tuple<std::array<Sphere, SphereCount>, std::array<Light, LightCount>>
get_render_data()
{
	Material      ivory(1.0, Vec4f(0.6f,  0.3f, 0.1f, 0.0f), Vec3f(0.4f, 0.4f, 0.3f),   50.0f);
    Material      glass(1.5, Vec4f(0.0f,  0.5f, 0.1f, 0.8f), Vec3f(0.6f, 0.7f, 0.8f),  125.0f);
    Material red_rubber(1.0, Vec4f(0.9f,  0.1f, 0.0f, 0.0f), Vec3f(0.3f, 0.1f, 0.1f),   10.0f);
    Material     mirror(1.0, Vec4f(0.0f, 10.0f, 0.8f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f), 1425.0f);

    std::array<Sphere, SphereCount> spheres = {
		Sphere(Vec3f(-3.0f,  0.0f, -16.0f), 2.0f,      ivory),
		Sphere(Vec3f(-1.0f, -1.5f, -12.0f), 2.0f,      glass),
		Sphere(Vec3f( 1.5f, -0.5f, -18.0f), 3.0f, red_rubber),
		Sphere(Vec3f( 7.0f,  5.0f, -18.0f), 4.0f,     mirror)
	};

    std::array<Light, LightCount>  lights = {
		Light(Vec3f(-20.0f, 20.0f,  20.0f), 1.5f),
		Light(Vec3f( 30.0f, 50.0f, -25.0f), 1.8f),
		Light(Vec3f( 30.0f, 20.0f,  30.0f), 1.7f)
	};
	return make_tuple(spheres, lights);
}
#pragma region Raytracer code
constexpr Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}
constexpr Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - details::sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}
template <size_t SphereCount> constexpr bool scene_intersect(
	const Vec3f &orig, const Vec3f &dir, 
	const std::array<Sphere, SphereCount> &spheres, 
	Vec3f &hit, Vec3f &N, Material &material) 
{
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i = 0;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    float checkerboard_dist = std::numeric_limits<float>::max();
    if (details::fabs(dir.y) > 1e-3)  {
        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir*d;
        if (d>0 && fabs(pt.x)<10 && pt.z<-10 && pt.z>-30 && d<spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0,1,0);
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
        }
    }
    return std::min(spheres_dist, checkerboard_dist)<1000;
}
template <size_t SphereCount, size_t LightCount> constexpr 
Vec3f cast_ray(
	const Vec3f &orig, const Vec3f &dir, 
	const std::array<Sphere, SphereCount> &spheres, 
	const std::array<Light, LightCount> &lights, 
	size_t depth=0) 
{
    Vec3f point = {}, N = {};
    Material material = {};

    if (depth>4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        return Vec3f(0.2f, 0.7f, 0.8f); // background color
    }

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);

        float te1 = std::max(0.f, -reflect(-light_dir, N)*dir);
        float te2 = material.specular_exponent;
        //printf("pow(%f,%f)\n",te1,te2);
        specular_light_intensity += details::powf(te1, te2)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}
template <size_t SphereCount, size_t LightCount> constexpr 
std::array<pixel, W*H> render(
	const std::tuple<std::array<Sphere, SphereCount>, std::array<Light, LightCount>>& render_data) 
{
    const float fov      = M_PI/3.0f;
	std::array<pixel, W*H> framebuffer = {};
	auto [spheres, lights] = render_data;

    #pragma omp parallel for
    for (size_t j = 0; j<H; j++) { // actual rendering loop
        for (size_t i = 0; i<W; i++) {
            float dir_x =  (i + 0.5) -  W/2.;
            float dir_y = -(j + 0.5) + H/2.;    // this flips the image at the same time
            float dir_z = -H/(2.*details::tanf(fov/2.));
            framebuffer[i+(H-j-1)*W] = cast_ray(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }
	return framebuffer;
}
#pragma endregion

#pragma region BMP data
#pragma pack(push)
#pragma pack(1)
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
	std::array<pixel, W*H> data;
};
struct bmp
{
	bmp_fileheader hfile;
	bmp_infoheader hinfo;
	bmp_data hdata;
};
#pragma pack(pop)
#pragma endregion
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
	bmp_data res = { render(get_render_data()) };
	/*for (size_t j = 0; j < H; ++j)
		for (size_t i = 0; i < W; ++i)
		{
			uint8_t a = 0xFF, r = 0, g = 0, b = 0;
			if (i < W / 3) r = (uint8_t)( ( ((float)i) / W * 3.0f) * 255);
			else if (i < 2 * W / 3) g = (uint8_t)( ( ((float)(i - W / 3)) / W * 3.0f) * 255);
			else b = (uint8_t)( ( ((float)(i - 2 * W / 3)) / W * 3.0f) * 255);
			res.data[j * W + i].a = a;
			res.data[j * W + i].r = r;
			res.data[j * W + i].g = g;
			res.data[j * W + i].b = b;
		}*/
	return res;
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
		r[14+40+i] = val.hdata.data[i / 4].b;
		r[14+40+i + 1] = val.hdata.data[i / 4].g;
		r[14+40+i + 2] = val.hdata.data[i / 4].r;
		r[14+40+i + 3] = val.hdata.data[i / 4].a;
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
	printf("done\n");
	return 0;
}