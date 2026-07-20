#include <windows.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include "filter2.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr wchar_t kSourceResource[] = L"resource:WindSwayT.Source";

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

template<typename Real, bool Optimized>
void build_geometry_impl(
    int width, int height, double time, int instance, Geometry& geometry) {
    geometry.vertices.clear();
    const int n0 = std::clamp(static_cast<int>(std::lround(divisions.value)), 2, 300);
    int n = n0;
    Real top = static_cast<Real>(std::clamp(fixed_top.value, 0.0, 100.0));
    Real bottom = static_cast<Real>(std::clamp(
        fixed_bottom.value, 0.0, 100.0 - static_cast<double>(top)));
    if (lower_base.value) std::swap(top, bottom);
    top *= static_cast<Real>(0.01 * height);
    bottom *= static_cast<Real>(0.01 * height);

    const Real flexible_length = static_cast<Real>(height) - top - bottom;
    const Real half_width = static_cast<Real>(width) * Real{0.5};
    const Real half_height = static_cast<Real>(height) * Real{0.5};
    const Real period = static_cast<Real>(std::max(0.01, sway_period.value));
    const Real phase_rate = static_cast<Real>(2.0 * kPi) / period;
    const Real phase_step = static_cast<Real>(2.0 * sway_offset.value * kPi / 180.0);
    const Real center_radians = static_cast<Real>(center.value * kPi / 180.0);
    Real amplitude = static_cast<Real>(sway_angle.value * kPi / 180.0);
    const Real local_time = static_cast<Real>(time - instance * time_shift.value);

    if (random_sway.value) {
        double s = static_cast<double>(local_time / period);
        const int i1 = static_cast<int>(std::floor(s));
        const std::array<int, 4> source{i1 - 1, i1, i1 + 1, i1 + 2};
        std::array<double, 4> f;
        const int seed = 10 + std::abs(static_cast<int>(std::lround(random_pattern.value)));
        for (int i = 0; i < 4; ++i) {
            const int m = source[i] < 0 ? 10000 - source[i] : source[i] + 1;
            f[i] = deterministic_random(-seed, m) * amplitude;
        }
        s -= i1;
        amplitude = static_cast<Real>(catmull_rom(s, f[0], f[1], f[2], f[3]));
    }

    std::array<Real, 303> x, y, v;
    x[0] = Real{0};
    y[0] = Real{0};
    const Real segment = flexible_length / static_cast<Real>(n0);
    const Real phase = phase_rate * local_time;
    if constexpr (Optimized) {
        const Real inv_n0 = Real{1} / static_cast<Real>(n0);
        const Real phase_step_per_i = phase_step * inv_n0;
        const Real step_sin = std::sin(phase_step_per_i);
        const Real step_cos = std::cos(phase_step_per_i);
        Real wave_sin = std::sin(phase - phase_step_per_i);
        Real wave_cos = std::cos(phase - phase_step_per_i);
        for (int i = 1; i <= n0; ++i) {
            const Real ratio = static_cast<Real>(i) * inv_n0;
            const Real taper = Real{1} - ratio;
            const Real taper_squared = taper * taper;
            const Real angle = (amplitude * wave_sin + center_radians)
                * (Real{1} - taper_squared * taper_squared);
            x[i] = x[i - 1] + std::sin(angle);
            y[i] = y[i - 1] + std::cos(angle);

            if (i < n0) {
                if ((i & 63) == 0) {
                    const Real next_phase = phase - static_cast<Real>(i + 1) * phase_step_per_i;
                    wave_sin = std::sin(next_phase);
                    wave_cos = std::cos(next_phase);
                } else {
                    const Real next_sin = wave_sin * step_cos - wave_cos * step_sin;
                    wave_cos = wave_cos * step_cos + wave_sin * step_sin;
                    wave_sin = next_sin;
                }
            }
        }

        x[0] *= segment;
        y[0] = y[0] * segment + top - half_height;
        v[0] = top;
        for (int i = 1; i <= n0; ++i) {
            x[i] *= segment;
            y[i] = y[i] * segment + top - half_height;
            v[i] = v[i - 1] + segment;
        }
    } else {
        for (int i = 1; i <= n0; ++i) {
            const Real ratio = static_cast<Real>(i) / static_cast<Real>(n0);
            const Real angle = (amplitude * std::sin(
                phase - static_cast<Real>(i) * phase_step / static_cast<Real>(n0))
                + center_radians) * (Real{1} - std::pow(Real{1} - ratio, Real{4}));
            x[i] = x[i - 1] + std::sin(angle);
            y[i] = y[i - 1] + std::cos(angle);
        }
        for (int i = 0; i <= n0; ++i) {
            x[i] *= segment;
            y[i] = y[i] * segment + top - half_height;
            v[i] = segment * i + top;
        }
    }

    std::array<Real, 303> x1, x2, y1, y2;
    x1[0] = -half_width;
    x2[0] = half_width;
    y1[0] = y2[0] = -half_height;
    const auto build_edges = [&](auto length_of) {
        for (int i = 1; i <= n0 + 1; ++i) {
            const Real px = (i == 1 ? Real{0} : x[i - 2]);
            const Real py = (i == 1 ? -half_height : y[i - 2]);
            const Real cx = x[i - 1];
            const Real cy = y[i - 1];
            Real dx = -(cy - py);
            Real dy = cx - px;
            const Real length = length_of(dx, dy);
            if (length > Real{0}) {
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
    };
    if constexpr (Optimized) {
        build_edges([](Real dx, Real dy) { return std::sqrt(dx * dx + dy * dy); });
    } else {
        build_edges([](Real dx, Real dy) { return std::hypot(dx, dy); });
    }

    if (bottom > Real{0}) {
        Real dx = x[n0] - x[n0 - 1];
        Real dy = y[n0] - y[n0 - 1];
        Real length;
        if constexpr (Optimized) length = std::sqrt(dx * dx + dy * dy);
        else length = std::hypot(dx, dy);
        if (length > Real{0}) { dx *= bottom / length; dy *= bottom / length; }
        x1[n0 + 1] = x1[n0] + dx; x2[n0 + 1] = x2[n0] + dx;
        y1[n0 + 1] = y1[n0] + dy; y2[n0 + 1] = y2[n0] + dy;
        v[n0 + 1] = height;
        n = n0 + 1;
    }

    Real min_x = -half_width, max_x = half_width;
    Real min_y = -half_height, max_y = -half_height;
    for (int i = 0; i <= n; ++i) {
        min_x = std::min({min_x, x1[i], x2[i]});
        max_x = std::max({max_x, x1[i], x2[i]});
        min_y = std::min({min_y, y1[i], y2[i]});
        max_y = std::max({max_y, y1[i], y2[i]});
    }

    geometry.center_x = static_cast<double>((max_x + min_x) * Real{0.5});
    geometry.center_y = static_cast<double>((max_y + min_y) * Real{0.5});
    geometry.width = std::max(1, static_cast<int>(std::ceil(max_x - min_x)));
    geometry.height = std::max(1, static_cast<int>(std::ceil(max_y - min_y)));
    const bool use_breakage_reduction = reduce_breakage.value;
    geometry.vertices.reserve(static_cast<std::size_t>(n + 1) * (use_breakage_reduction ? 16u : 4u));

    if (!use_breakage_reduction) {
        for (int i = 0; i <= n; ++i) {
            const Real ax = (i == 0 ? -half_width : x1[i - 1]) - static_cast<Real>(geometry.center_x);
            const Real ay = (i == 0 ? -half_height : y1[i - 1]) - static_cast<Real>(geometry.center_y);
            const Real bx = (i == 0 ? half_width : x2[i - 1]) - static_cast<Real>(geometry.center_x);
            const Real by = (i == 0 ? -half_height : y2[i - 1]) - static_cast<Real>(geometry.center_y);
            const Real cx = x2[i] - static_cast<Real>(geometry.center_x);
            const Real cy = y2[i] - static_cast<Real>(geometry.center_y);
            const Real dx = x1[i] - static_cast<Real>(geometry.center_x);
            const Real dy = y1[i] - static_cast<Real>(geometry.center_y);
            const Real vt = i == 0 ? Real{0} : v[i - 1];
            const Real vb = v[i];
            append_quad(geometry.vertices, ax, ay, bx, by, cx, cy, dx, dy,
                vt, vb, width, height, lower_base.value);
        }
    } else {
        for (int i = 0; i <= n; ++i) {
            const Real ax = (i == 0 ? -half_width : x1[i - 1]) - static_cast<Real>(geometry.center_x);
            const Real ay = (i == 0 ? -half_height : y1[i - 1]) - static_cast<Real>(geometry.center_y);
            const Real bx = (i == 0 ? half_width : x2[i - 1]) - static_cast<Real>(geometry.center_x);
            const Real by = (i == 0 ? -half_height : y2[i - 1]) - static_cast<Real>(geometry.center_y);
            const Real cx = x2[i] - static_cast<Real>(geometry.center_x);
            const Real cy = y2[i] - static_cast<Real>(geometry.center_y);
            const Real dx = x1[i] - static_cast<Real>(geometry.center_x);
            const Real dy = y1[i] - static_cast<Real>(geometry.center_y);
            const Real vt = i == 0 ? Real{0} : v[i - 1];
            const Real vb = v[i];
            const Real mx = (ax + bx + cx + dx) * Real{0.25};
            const Real my = (ay + by + cy + dy) * Real{0.25};
            const Real vm = (vt + vb) * Real{0.5};
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
}

void build_geometry(
    int width, int height, double time, int instance, Geometry& geometry) {
    build_geometry_impl<float, true>(width, height, time, instance, geometry);
}

inline void configure_warp_rendering(FILTER_PROC_VIDEO* video) {
    video->set_sampler_mode(SAMPLER_MODE::CLAMP);
    video->set_culling_state(false);
    video->set_blend_mode(BLEND_MODE::NONE);
}

void append_repeat_vertices(
    std::vector<VERTEX_TEXTURE>& output, const Geometry& geometry,
    int instance, int count) {
    const float offset_x = static_cast<float>(
        (instance - (count + 1) * 0.5) * repeat_spacing.value + geometry.center_x);
    const bool flip = lower_base.value;
    const float offset_y = static_cast<float>(flip ? -geometry.center_y : geometry.center_y);
    const float scale_y = flip ? -1.0f : 1.0f;
    for (const auto& source : geometry.vertices) {
        auto vertex = source;
        vertex.x += offset_x;
        vertex.y = vertex.y * scale_y + offset_y;
        output.push_back(vertex);
    }
}

std::vector<VERTEX_TEXTURE>& build_repeat_vertices(
    int source_width, int source_height, double time, int count) {
    thread_local Geometry geometry;
    thread_local std::vector<VERTEX_TEXTURE> repeat_vertices;
    repeat_vertices.clear();
    const std::size_t vertices_per_instance =
        static_cast<std::size_t>(std::clamp(
            static_cast<int>(std::lround(divisions.value)), 2, 300) + 2)
        * (reduce_breakage.value ? 16u : 4u);
    repeat_vertices.reserve(vertices_per_instance * static_cast<std::size_t>(count));

    if (time_shift.value == 0.0) {
        build_geometry(source_width, source_height, time, 1, geometry);
        repeat_vertices.reserve(geometry.vertices.size() * static_cast<std::size_t>(count));
        for (int i = 1; i <= count; ++i) {
            append_repeat_vertices(repeat_vertices, geometry, i, count);
        }
    } else {
        for (int i = 1; i <= count; ++i) {
            build_geometry(source_width, source_height, time, i, geometry);
            append_repeat_vertices(repeat_vertices, geometry, i, count);
        }
    }
    return repeat_vertices;
}

bool render_normal(FILTER_PROC_VIDEO* video, int source_width, int source_height) {
    thread_local Geometry geometry;
    build_geometry(source_width, source_height, video->object->time, 1, geometry);
    video->create_image_resource(L"object", nullptr, geometry.width, geometry.height);
    video->clear_image_resource(L"object", PIXEL_RGBA{0, 0, 0, 0});
    configure_warp_rendering(video);
    if (!video->draw_poly_to_resource(
        L"object", VERTEX_TYPE::QUAD_TEXTURE,
        geometry.vertices.data(), static_cast<int>(geometry.vertices.size()),
        kSourceResource)) return false;

    video->param->cx = static_cast<float>(-geometry.center_x);
    video->param->cy = static_cast<float>(
        lower_base.value ? geometry.center_y : -geometry.center_y);
    return true;
}

bool render_repeat_direct(
    FILTER_PROC_VIDEO* video, int source_width, int source_height, int count) {
    auto& repeat_vertices = build_repeat_vertices(
        source_width, source_height, video->object->time, count);

    configure_warp_rendering(video);
    if (!video->draw_poly(
        VERTEX_TYPE::QUAD_TEXTURE, repeat_vertices.data(),
        static_cast<int>(repeat_vertices.size()), kSourceResource)) return false;

    static constexpr PIXEL_RGBA transparent{0, 0, 0, 0};
    video->set_image_data(&transparent, 1, 1);
    return true;
}

bool process_video(FILTER_PROC_VIDEO* video) {
    if (!video || !video->object || video->object->width <= 0 || video->object->height <= 0) return true;
    const int source_width = video->object->width;
    const int source_height = video->object->height;
    const int count = repeat_horizontal.value
        ? std::clamp(static_cast<int>(std::lround(repeat_count.value)), 1, 50) : 1;
    const bool repeat = repeat_horizontal.value;

    if (!video->copy_image_resource(kSourceResource, L"object")) return false;
    if (!repeat) {
        return render_normal(video, source_width, source_height);
    }
    return render_repeat_direct(video, source_width, source_height, count);
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
