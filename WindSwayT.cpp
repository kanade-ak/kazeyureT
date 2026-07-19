#include <windows.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include "filter2.h"
#include "normalize_shader.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr wchar_t kSourceResource[] = L"resource:WindSwayT.Source";
constexpr wchar_t kWarpResource[] = L"tempbuffer";

auto sway_angle = FILTER_ITEM_TRACK(L"揺れ角", 30.0, 0.0, 360.0, 0.1);
auto sway_period = FILTER_ITEM_TRACK(L"揺れ周期", 2.0, 0.01, 100.0, 0.01);
auto sway_offset = FILTER_ITEM_TRACK(L"揺れズレ", 90.0, -360.0, 360.0, 0.1);
auto center = FILTER_ITEM_TRACK(L"センター", 0.0, -180.0, 180.0, 0.1);
auto divisions = FILTER_ITEM_TRACK(L"分割数", 10.0, 2.0, 300.0, 1.0);
auto fixed_top = FILTER_ITEM_TRACK(L"上固定長％", 10.0, 0.0, 100.0, 0.1);
auto fixed_bottom = FILTER_ITEM_TRACK(L"下固定長％", 10.0, 0.0, 100.0, 0.1);
auto lower_base = FILTER_ITEM_CHECK(L"下を基準", false);
auto random_sway = FILTER_ITEM_CHECK(L"ランダム揺れ量", false);
auto random_pattern = FILTER_ITEM_TRACK(L"ランダム揺れパターン", 0.0, 0.0, 10000.0, 1.0);
auto time_shift = FILTER_ITEM_TRACK(L"時間ずれ", 0.1, -10.0, 10.0, 0.01);
auto repeat_horizontal = FILTER_ITEM_CHECK(L"横に繰り返す", false);
auto repeat_count = FILTER_ITEM_TRACK(L"繰り返し個数", 3.0, 1.0, 50.0, 1.0);
auto repeat_spacing = FILTER_ITEM_TRACK(L"間隔", 50.0, 0.0, 1000.0, 0.1);
auto reduce_breakage = FILTER_ITEM_CHECK(L"破綻軽減", false);
auto alpha_correction = FILTER_ITEM_CHECK(L"アルファ補正", true);

void* items[] = {
    &sway_angle, &sway_period, &sway_offset, &center, &divisions,
    &fixed_top, &fixed_bottom, &lower_base, &random_sway, &random_pattern,
    &time_shift, &repeat_horizontal, &repeat_count, &repeat_spacing,
    &reduce_breakage, &alpha_correction, nullptr
};

struct Geometry {
    std::vector<VERTEX_TEXTURE> vertices;
    int width = 1;
    int height = 1;
    double center_x = 0.0;
    double center_y = 0.0;
};

[[nodiscard]] inline std::uint32_t mix32(std::uint32_t x) noexcept {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    return x ^ (x >> 16);
}

[[nodiscard]] inline double deterministic_random(int seed0, int seed1) noexcept {
    const auto a = static_cast<std::uint32_t>(seed0);
    const auto b = static_cast<std::uint32_t>(seed1);
    return static_cast<double>(mix32(a ^ std::rotl(b, 16) ^ 0x9e3779b9u)) / 4294967295.0;
}

[[nodiscard]] inline double catmull_rom(
    double t, double p0, double p1, double p2, double p3) noexcept {
    const double t2 = t * t;
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t2 * t);
}

inline void append_quad(
    std::vector<VERTEX_TEXTURE>& out,
    double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3,
    double v0, double v1, double width, double height,
    bool flip_vertical) {
    const float tv0 = static_cast<float>((flip_vertical ? height - v0 : v0) / height);
    const float tv1 = static_cast<float>((flip_vertical ? height - v1 : v1) / height);
    out.push_back({static_cast<float>(x0), static_cast<float>(flip_vertical ? -y0 : y0), 0.0f, 0.0f, tv0, 1.0f});
    out.push_back({static_cast<float>(x1), static_cast<float>(flip_vertical ? -y1 : y1), 0.0f, 1.0f, tv0, 1.0f});
    out.push_back({static_cast<float>(x2), static_cast<float>(flip_vertical ? -y2 : y2), 0.0f, 1.0f, tv1, 1.0f});
    out.push_back({static_cast<float>(x3), static_cast<float>(flip_vertical ? -y3 : y3), 0.0f, 0.0f, tv1, 1.0f});
}

inline void append_vertex(
    std::vector<VERTEX_TEXTURE>& out, double x, double y,
    double u, double v, double width, double height, bool flip_vertical) {
    out.push_back({
        static_cast<float>(x),
        static_cast<float>(flip_vertical ? -y : y),
        0.0f,
        static_cast<float>(u / width),
        static_cast<float>((flip_vertical ? height - v : v) / height),
        1.0f
    });
}

inline void append_triangle_as_quad(
    std::vector<VERTEX_TEXTURE>& out,
    double x0, double y0, double u0, double v0,
    double x1, double y1, double u1, double v1,
    double x2, double y2, double u2, double v2,
    double width, double height, bool flip_vertical) {
    append_vertex(out, x0, y0, u0, v0, width, height, flip_vertical);
    append_vertex(out, x1, y1, u1, v1, width, height, flip_vertical);
    append_vertex(out, x2, y2, u2, v2, width, height, flip_vertical);
    append_vertex(out, x2, y2, u2, v2, width, height, flip_vertical);
}

[[nodiscard]] Geometry build_geometry(int width, int height, double time, int instance) {
    const int n0 = std::clamp(static_cast<int>(std::lround(divisions.value)), 2, 300);
    int n = n0;
    double top = std::clamp(fixed_top.value, 0.0, 100.0);
    double bottom = std::clamp(fixed_bottom.value, 0.0, 100.0 - top);
    if (lower_base.value) std::swap(top, bottom);
    top *= 0.01 * height;
    bottom *= 0.01 * height;

    const double flexible_length = height - top - bottom;
    const double half_width = width * 0.5;
    const double half_height = height * 0.5;
    const double period = std::max(0.01, sway_period.value);
    const double phase_rate = 2.0 * kPi / period;
    const double phase_step = 2.0 * sway_offset.value * kPi / 180.0;
    const double center_radians = center.value * kPi / 180.0;
    double amplitude = sway_angle.value * kPi / 180.0;
    const double local_time = time - instance * time_shift.value;

    if (random_sway.value) {
        double s = local_time / period;
        const int i1 = static_cast<int>(std::floor(s));
        const std::array<int, 4> source{i1 - 1, i1, i1 + 1, i1 + 2};
        std::array<double, 4> f{};
        const int seed = 10 + std::abs(static_cast<int>(std::lround(random_pattern.value)));
        for (int i = 0; i < 4; ++i) {
            const int m = source[i] < 0 ? 10000 - source[i] : source[i] + 1;
            f[i] = deterministic_random(-seed, m) * amplitude;
        }
        s -= i1;
        amplitude = catmull_rom(s, f[0], f[1], f[2], f[3]);
    }

    std::array<double, 303> x{}, y{}, v{};
    const double segment = flexible_length / n0;
    for (int i = 1; i <= n0; ++i) {
        const double ratio = static_cast<double>(i) / n0;
        const double angle = (amplitude * std::sin(phase_rate * local_time - i * phase_step / n0)
            + center_radians) * (1.0 - std::pow(1.0 - ratio, 4.0));
        x[i] = x[i - 1] + std::sin(angle);
        y[i] = y[i - 1] + std::cos(angle);
    }
    for (int i = 0; i <= n0; ++i) {
        x[i] *= segment;
        y[i] = y[i] * segment + top - half_height;
        v[i] = segment * i + top;
    }

    std::array<double, 303> x1{}, x2{}, y1{}, y2{};
    x1[0] = -half_width;
    x2[0] = half_width;
    y1[0] = y2[0] = -half_height;
    for (int i = 1; i <= n0 + 1; ++i) {
        const double px = (i == 1 ? 0.0 : x[i - 2]);
        const double py = (i == 1 ? -half_height : y[i - 2]);
        const double cx = x[i - 1];
        const double cy = y[i - 1];
        double dx = -(cy - py);
        double dy = cx - px;
        const double length = std::hypot(dx, dy);
        if (length > 0.0) {
            dx *= half_width / length;
            dy *= half_width / length;
            x1[i - 1] = cx + dx;
            x2[i - 1] = cx - dx;
            y1[i - 1] = cy + dy;
            y2[i - 1] = cy - dy;
        } else if (i > 1) {
            x1[i - 1] = x1[i - 2]; x2[i - 1] = x2[i - 2];
            y1[i - 1] = y1[i - 2]; y2[i - 1] = y2[i - 2];
        }
    }

    if (bottom > 0.0) {
        double dx = x[n0] - x[n0 - 1];
        double dy = y[n0] - y[n0 - 1];
        const double length = std::hypot(dx, dy);
        if (length > 0.0) { dx *= bottom / length; dy *= bottom / length; }
        x1[n0 + 1] = x1[n0] + dx; x2[n0 + 1] = x2[n0] + dx;
        y1[n0 + 1] = y1[n0] + dy; y2[n0 + 1] = y2[n0] + dy;
        v[n0 + 1] = height;
        n = n0 + 1;
    }

    double min_x = -half_width, max_x = half_width;
    double min_y = -half_height, max_y = -half_height;
    for (int i = 0; i <= n; ++i) {
        min_x = std::min({min_x, x1[i], x2[i]});
        max_x = std::max({max_x, x1[i], x2[i]});
        min_y = std::min({min_y, y1[i], y2[i]});
        max_y = std::max({max_y, y1[i], y2[i]});
    }

    Geometry geometry;
    geometry.center_x = (max_x + min_x) * 0.5;
    geometry.center_y = (max_y + min_y) * 0.5;
    geometry.width = std::max(1, static_cast<int>(std::ceil(max_x - min_x)));
    geometry.height = std::max(1, static_cast<int>(std::ceil(max_y - min_y)));
    geometry.vertices.reserve(static_cast<std::size_t>(n + 1) * (reduce_breakage.value ? 16u : 4u));

    for (int i = 0; i <= n; ++i) {
        const double ax = (i == 0 ? -half_width : x1[i - 1]) - geometry.center_x;
        const double ay = (i == 0 ? -half_height : y1[i - 1]) - geometry.center_y;
        const double bx = (i == 0 ? half_width : x2[i - 1]) - geometry.center_x;
        const double by = (i == 0 ? -half_height : y2[i - 1]) - geometry.center_y;
        const double cx = x2[i] - geometry.center_x;
        const double cy = y2[i] - geometry.center_y;
        const double dx = x1[i] - geometry.center_x;
        const double dy = y1[i] - geometry.center_y;
        const double vt = i == 0 ? 0.0 : v[i - 1];
        const double vb = v[i];
        if (!reduce_breakage.value) {
            append_quad(geometry.vertices, ax, ay, bx, by, cx, cy, dx, dy,
                vt, vb, width, height, lower_base.value);
        } else {
            const double mx = (ax + bx + cx + dx) * 0.25;
            const double my = (ay + by + cy + dy) * 0.25;
            const double vm = (vt + vb) * 0.5;
            append_triangle_as_quad(geometry.vertices,
                ax, ay, 0.0, vt, bx, by, width, vt, mx, my, half_width, vm,
                width, height, lower_base.value);
            append_triangle_as_quad(geometry.vertices,
                bx, by, width, vt, cx, cy, width, vb, mx, my, half_width, vm,
                width, height, lower_base.value);
            append_triangle_as_quad(geometry.vertices,
                dx, dy, 0.0, vb, ax, ay, 0.0, vt, mx, my, half_width, vm,
                width, height, lower_base.value);
            append_triangle_as_quad(geometry.vertices,
                cx, cy, width, vb, dx, dy, 0.0, vb, mx, my, half_width, vm,
                width, height, lower_base.value);
        }
    }
    return geometry;
}

bool render_geometry(FILTER_PROC_VIDEO* video, const Geometry& geometry) {
    video->create_image_resource(kWarpResource, nullptr, geometry.width, geometry.height);
    video->clear_image_resource(kWarpResource, PIXEL_RGBA{0, 0, 0, 0});
    video->set_sampler_mode(SAMPLER_MODE::CLAMP);
    video->set_culling_state(false);
    video->set_blend_mode(BLEND_MODE::NONE);
    return video->draw_poly_to_resource(
        kWarpResource, VERTEX_TYPE::QUAD_TEXTURE,
        geometry.vertices.data(), static_cast<int>(geometry.vertices.size()), kSourceResource);
}

bool process_video(FILTER_PROC_VIDEO* video) {
    if (!video || !video->object || video->object->width <= 0 || video->object->height <= 0) return true;
    const int source_width = video->object->width;
    const int source_height = video->object->height;
    if (!video->copy_image_resource(kSourceResource, L"object")) return false;

    const int count = repeat_horizontal.value
        ? std::clamp(static_cast<int>(std::lround(repeat_count.value)), 1, 50) : 1;

    if (count == 1 && !repeat_horizontal.value) {
        const auto geometry = build_geometry(source_width, source_height, video->object->time, 1);
        if (!render_geometry(video, geometry)) return false;
        video->create_image_resource(L"object", nullptr, geometry.width, geometry.height);
        if (alpha_correction.value) {
            LPCWSTR resources[] = { kWarpResource };
            if (!video->exec_pixelshader_data(
                g_normalize_shader, sizeof(g_normalize_shader), L"object", resources, 1,
                nullptr, 0, video->get_blend_state(BLEND_STATE_MODE::COPY), nullptr)) return false;
        } else if (!video->copy_image_resource(L"object", kWarpResource)) {
            return false;
        }
        video->param->cx = static_cast<float>(-geometry.center_x);
        video->param->cy = static_cast<float>(lower_base.value ? geometry.center_y : -geometry.center_y);
        return true;
    }

    for (int i = 1; i <= count; ++i) {
        const auto geometry = build_geometry(source_width, source_height, video->object->time, i);
        if (!render_geometry(video, geometry)) return false;
        const float x = static_cast<float>((i - (count + 1) * 0.5) * repeat_spacing.value + geometry.center_x);
        const float y = static_cast<float>(lower_base.value ? -geometry.center_y : geometry.center_y);
        if (!video->draw_image(kWarpResource, x, y, 0.0f, 0.0f, 0.0f, 0.0f,
            1.0f, lower_base.value ? -1.0f : 1.0f, 1.0f, 1.0f)) return false;
    }
    static constexpr PIXEL_RGBA transparent{0, 0, 0, 0};
    video->set_image_data(&transparent, 1, 1);
    return true;
}

FILTER_PLUGIN_TABLE plugin_table = {
    FILTER_PLUGIN_TABLE::FLAG_VIDEO,
    L"風揺れT(DLL)",
    L"風揺れT(DLL)",
    L"風揺れTの最適化",
    items,
    process_video,
    nullptr
};

} // namespace

EXTERN_C __declspec(dllexport) bool InitializePlugin(DWORD) {
    return true;
}

EXTERN_C __declspec(dllexport) void UninitializePlugin() {}

EXTERN_C __declspec(dllexport) FILTER_PLUGIN_TABLE* GetFilterPluginTable() {
    return &plugin_table;
}
