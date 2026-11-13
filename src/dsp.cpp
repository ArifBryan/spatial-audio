// libspatial-dsp.so: the Ambisonics renderer used by the spatial-audio filter graph (a LADSPA plugin for PipeWire's filter-chain).
//
// ambirot, order 3 (16 channels). The HRTF was fitted into spherical-harmonic channels offline (`spatial-tools gen-ambi`),
// so the filters are FIXED. At runtime each virtual speaker is encoded at its current direction in the head's frame (the head
// rotation is simply where the speaker is in that frame) and the channels are filtered with the fixed filters. Moving the
// head changes smooth encoding gains only: nothing is switched or interpolated between measurements. The interaural time
// difference is fitted in the same basis and applied with a smooth fractional delay per ear.
//
// The direction arrives as a unit vector (x forward, y left, z up). Angles are never used, so straight up/down is not special.
// A block limiter keeps the output below full scale: turning towards a speaker raises the level at the near ear.
//
// Ports: 0 "In 1" 1 "In 2" (left/right channel = speaker 1/2)   2 "Out 1" 3 "Out 2" (left/right ear)
//        4..6 "X 1" "Y 1" "Z 1" (speaker 1 direction)   7..9 "X 2" "Y 2" "Z 2"   10 "Mix" (0 = plain stereo, 1 = rendered)
// Data: <plugin folder>/../data/ambi/ambi.bin
#if __has_include(<ladspa.h>)
#include <ladspa.h>
#else  // the LADSPA interface is small and stable; this is the subset we need (same layout as ladspa.h)
extern "C" {
typedef float LADSPA_Data; typedef int LADSPA_Properties, LADSPA_PortDescriptor, LADSPA_PortRangeHintDescriptor; typedef void* LADSPA_Handle;
#define LADSPA_PORT_INPUT 0x1
#define LADSPA_PORT_OUTPUT 0x2
#define LADSPA_PORT_CONTROL 0x4
#define LADSPA_PORT_AUDIO 0x8
#define LADSPA_HINT_BOUNDED_BELOW 0x1
#define LADSPA_HINT_BOUNDED_ABOVE 0x2
#define LADSPA_HINT_DEFAULT_0 0x200
#define LADSPA_HINT_DEFAULT_1 0x240
#define LADSPA_HINT_DEFAULT_MAXIMUM 0x280
struct LADSPA_PortRangeHint { LADSPA_PortRangeHintDescriptor HintDescriptor; LADSPA_Data LowerBound, UpperBound; };
struct LADSPA_Descriptor {
    unsigned long UniqueID; const char *Label; LADSPA_Properties Properties; const char *Name, *Maker, *Copyright;
    unsigned long PortCount; const LADSPA_PortDescriptor* PortDescriptors; const char* const* PortNames; const LADSPA_PortRangeHint* PortRangeHints;
    void* ImplementationData;
    LADSPA_Handle (*instantiate)(const LADSPA_Descriptor*, unsigned long);
    void (*connect_port)(LADSPA_Handle, unsigned long, LADSPA_Data*);
    void (*activate)(LADSPA_Handle);
    void (*run)(LADSPA_Handle, unsigned long);
    void (*run_adding)(LADSPA_Handle, unsigned long);
    void (*set_run_adding_gain)(LADSPA_Handle, LADSPA_Data);
    void (*deactivate)(LADSPA_Handle);
    void (*cleanup)(LADSPA_Handle);
};
}
#endif

#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sh.h"

namespace {
// Dot product of two float arrays: eight independent accumulators let the compiler vectorise it; a wider (AVX2/FMA) version
// is picked at run time where the CPU has it. This is the hot loop of the renderer.
__attribute__((target_clones("avx2,fma", "default"))) float dotf(const float* a, const float* b, int n) {
    float s[8] = {0, 0, 0, 0, 0, 0, 0, 0}; int i = 0;
    for (; i + 8 <= n; i += 8) for (int k = 0; k < 8; ++k) s[k] += a[i + k] * b[i + k];
    float t = 0;
    for (; i < n; ++i) t += a[i] * b[i];
    for (int k = 0; k < 8; ++k) t += s[k];
    return t;
}

constexpr int RB = 64, RING = 1 << 13;                // processing block (1.3 ms at 48 kHz); delay ring (170 ms, ample for the time difference)
constexpr float CEILING = 0.95f;                      // the limiter keeps peaks under this

struct AmbiData { uint32_t C = 0, T = 0; std::vector<float> itd, h; };            // h[(c*2+ear)*T + n], stored in file order
AmbiData& ambiData() {
    static AmbiData d; static bool tried = false;
    if (tried) return d;
    tried = true;
    Dl_info di; if (!dladdr((void*)&ambiData, &di) || !di.dli_fname) return d;
    std::string so = di.dli_fname, path = so.substr(0, so.rfind('/')) + "/../data/ambi/ambi.bin";
    FILE* f = fopen(path.c_str(), "rb"); if (!f) return d;
    char magic[8]; uint32_t hdr[2];
    if (fread(magic, 1, 8, f) != 8 || memcmp(magic, "SPAM1\0\0\0", 8) != 0 || fread(hdr, 4, 2, f) != 2 || hdr[0] != (uint32_t)SH_CHANNELS) { fclose(f); return d; }
    d.C = hdr[0]; d.T = hdr[1]; d.itd.resize(d.C); d.h.resize((size_t)d.C * 2 * d.T);
    bool ok = fread(d.itd.data(), 4, d.itd.size(), f) == d.itd.size() && fread(d.h.data(), 4, d.h.size(), f) == d.h.size();
    fclose(f); if (!ok) d = AmbiData(); return d;
}

struct Ambi {
    double rate = 48000.0; bool primed = false; AmbiData* data = nullptr; int T = 0;
    const float *in[2] = {nullptr, nullptr}, *ctl[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}; float* out[2] = {nullptr, nullptr};
    double y[2][SH_CHANNELS], curDelay[2][2] = {{0, 0}, {0, 0}}, mix = 1.0, gain = 1.0;   // encoding gains per speaker; smoothed delays [speaker][ear]; stereo/rendered mix; limiter gain
    std::vector<float> ring[2], hist[2][SH_CHANNELS], hr[SH_CHANNELS][2];              // delay rings per speaker; encoded history [ear][channel]; reversed filters
    unsigned pos = 0;
};
LADSPA_Handle ambiInstantiate(const LADSPA_Descriptor*, unsigned long rate) {
    auto* a = new Ambi; a->rate = (double)rate; a->data = &ambiData(); a->T = (int)a->data->T;
    for (int s = 0; s < 2; ++s) a->ring[s].assign(RING, 0.f);
    for (int c = 0; c < SH_CHANNELS; ++c) for (int e = 0; e < 2; ++e) {
        a->hist[e][c].assign(a->T + RB, 0.f); a->hr[c][e].assign(a->T, 0.f);
        if (a->data->C) for (int k = 0; k < a->T; ++k) a->hr[c][e][a->T - 1 - k] = a->data->h[((size_t)c * 2 + e) * a->T + k];     // reversed once
    }
    return a;
}
void ambiConnect(LADSPA_Handle h, unsigned long port, LADSPA_Data* p) {
    auto* a = (Ambi*)h;
    if (port < 2) a->in[port] = p; else if (port < 4) a->out[port - 2] = p; else a->ctl[port - 4] = p;
}
// Unit direction from the three control values (a degenerate vector falls back to straight ahead).
void direction(const float* const* c, double* x, double* y, double* z) {
    double a = *c[0], b = *c[1], d = *c[2], n = std::sqrt(a * a + b * b + d * d);
    if (n < 1e-6) { *x = 1; *y = 0; *z = 0; } else { *x = a / n; *y = b / n; *z = d / n; }
}
void ambiRun(LADSPA_Handle hd, unsigned long n) {
    auto* a = (Ambi*)hd; const int T = a->T;
    if (!a->data || a->data->C == 0 || T == 0) { for (int e = 0; e < 2; ++e) std::copy(a->in[e], a->in[e] + n, a->out[e]); return; }   // no data: plain stereo
    const double dl = 1.0 - std::exp(-1.0 / (0.015 * a->rate)), release = 1.0 - std::exp(-(double)RB / (0.15 * a->rate));
    for (unsigned long b0 = 0; b0 < n; b0 += RB) {
        const int b = (int)std::min<unsigned long>(RB, n - b0); double yt[2][SH_CHANNELS], tgt[2][2];
        for (int s = 0; s < 2; ++s) {
            double x, yy, z; direction(&a->ctl[3 * s], &x, &yy, &z);
            shEval(x, yy, z, yt[s]);
            double itd = 0; for (int c = 0; c < SH_CHANNELS; ++c) itd += a->data->itd[c] * yt[s][c];
            tgt[s][0] = std::fmax(0.0, -itd) * a->rate; tgt[s][1] = std::fmax(0.0, itd) * a->rate;
        }
        if (!a->primed) { for (int s = 0; s < 2; ++s) { for (int c = 0; c < SH_CHANNELS; ++c) a->y[s][c] = yt[s][c]; for (int e = 0; e < 2; ++e) a->curDelay[s][e] = tgt[s][e]; } a->mix = std::min(1.0, std::max(0.0, (double)*a->ctl[6])); a->primed = true; }
        double step[2][SH_CHANNELS]; for (int s = 0; s < 2; ++s) for (int c = 0; c < SH_CHANNELS; ++c) step[s][c] = (yt[s][c] - a->y[s][c]) / b;    // linear ramp over the block: continuous encoding
        for (int i = 0; i < b; ++i) {
            double xd[2][2];
            for (int s = 0; s < 2; ++s) {
                a->ring[s][(a->pos + i) & (RING - 1)] = a->in[s][b0 + i];
                for (int e = 0; e < 2; ++e) {
                    a->curDelay[s][e] += dl * (tgt[s][e] - a->curDelay[s][e]);
                    double rd = (double)(a->pos + i) - (a->curDelay[s][e] + 2.0), fl = std::floor(rd), t = rd - fl; long k = (long)fl; const std::vector<float>& R = a->ring[s];
                    float y0 = R[(k - 1) & (RING - 1)], y1 = R[k & (RING - 1)], y2 = R[(k + 1) & (RING - 1)], y3 = R[(k + 2) & (RING - 1)];
                    double c1 = 0.5 * (y2 - y0), c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3, c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
                    xd[s][e] = ((c3 * t + c2) * t + c1) * t + y1;
                }
            }
            for (int s = 0; s < 2; ++s) for (int c = 0; c < SH_CHANNELS; ++c) a->y[s][c] += step[s][c];
            for (int e = 0; e < 2; ++e) for (int c = 0; c < SH_CHANNELS; ++c) a->hist[e][c][T - 1 + i] = (float)(a->y[0][c] * xd[0][e] + a->y[1][c] * xd[1][e]);   // encode
        }
        float* o[2] = {a->out[0] + b0, a->out[1] + b0};
        for (int e = 0; e < 2; ++e) for (int i = 0; i < b; ++i) {
            float acc = 0;
            for (int c = 0; c < SH_CHANNELS; ++c) acc += dotf(a->hr[c][e].data(), &a->hist[e][c][i], T);
            o[e][i] = acc;                                                                  // fixed filters: sum of the filtered channels
        }
        const double mixTo = std::min(1.0, std::max(0.0, (double)*a->ctl[6])), mixStep = (mixTo - a->mix) / b;
        if (mixStep != 0.0 || a->mix < 1.0)                                                 // fade to / from plain stereo
            for (int i = 0; i < b; ++i) { double m = a->mix + mixStep * (i + 1); for (int e = 0; e < 2; ++e) o[e][i] = (float)(m * o[e][i] + (1.0 - m) * a->in[e][b0 + i]); }
        a->mix = mixTo;
        float peak = 0; for (int e = 0; e < 2; ++e) for (int i = 0; i < b; ++i) peak = std::max(peak, std::fabs(o[e][i]));
        double want = peak > CEILING ? CEILING / peak : 1.0, to = std::min(want, a->gain + (1.0 - a->gain) * release);   // down at once, back up slowly
        const int ramp = to < a->gain ? std::min(b, 16) : b; const double gs = (to - a->gain) / ramp;
        for (int i = 0; i < b; ++i) {
            double g = i < ramp ? a->gain + gs * (i + 1) : to;
            for (int e = 0; e < 2; ++e) o[e][i] = std::fmax(-1.f, std::fmin(1.f, (float)(o[e][i] * g)));
        }
        a->gain = to;
        for (int e = 0; e < 2; ++e) for (int c = 0; c < SH_CHANNELS; ++c) std::copy(a->hist[e][c].begin() + b, a->hist[e][c].begin() + b + T - 1, a->hist[e][c].begin());
        a->pos += b;
    }
}
void ambiCleanup(LADSPA_Handle h) { delete (Ambi*)h; }

LADSPA_PortDescriptor kPorts[11]; LADSPA_PortRangeHint kHints[11]; LADSPA_Descriptor kDescriptor;
const char* const kNames[11] = {"In 1", "In 2", "Out 1", "Out 2", "X 1", "Y 1", "Z 1", "X 2", "Y 2", "Z 2", "Mix"};
const LADSPA_Descriptor* descriptor() {
    static bool init = false;
    if (!init) {
        for (int i = 0; i < 11; ++i) {
            kPorts[i] = i < 2 ? (LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO) : i < 4 ? (LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO) : (LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL);
            kHints[i] = i < 4 ? LADSPA_PortRangeHint{0, 0, 0}
                      : i == 10 ? LADSPA_PortRangeHint{LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE | LADSPA_HINT_DEFAULT_1, 0.0f, 1.0f}
                      : LADSPA_PortRangeHint{LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE | LADSPA_HINT_DEFAULT_0, -1.0f, 1.0f};
        }
        kDescriptor = {0x53504436 /* "SPD6" */, "ambirot", 0, "Ambisonics HRTF renderer (spatial-audio)", "spatial-audio", "MIT", 11, kPorts, kNames, kHints, nullptr,
                       ambiInstantiate, ambiConnect, nullptr, ambiRun, nullptr, nullptr, nullptr, ambiCleanup};
        init = true;
    }
    return &kDescriptor;
}
}  // namespace

extern "C" const LADSPA_Descriptor* ladspa_descriptor(unsigned long index) { return index == 0 ? descriptor() : nullptr; }
