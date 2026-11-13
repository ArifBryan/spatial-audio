// Self-checks for the ambirot plugin in libspatial-dsp.so.  Usage: dsp <libspatial-dsp.so>
// Needs data/ambi/ambi.bin (`make hrtf`); without it the plugin passes stereo through and the rendering checks are skipped.
#include <dlfcn.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

struct D {   // same layout as LADSPA_Descriptor (see src/dsp.cpp)
    unsigned long id; const char* label; int props; const char *n, *m, *c; unsigned long pc; const int* pd; const char* const* pn; const void* ph; void* impl;
    void* (*inst)(const D*, unsigned long); void (*conn)(void*, unsigned long, float*); void (*act)(void*); void (*run)(void*, unsigned long); void* ra; void* sg; void* de; void (*clean)(void*);
};
static int failures = 0;
static void check(bool ok, const char* what, const char* detail) { printf("%-46s %s  %s\n", what, ok ? "ok  " : "FAIL", detail); if (!ok) ++failures; }

struct Run {   // one plugin instance fed block by block: 8 input channels (FL FR FC LFE RL RR SL SR), two ears out, head quaternion + Mix
    const D* d; void* h; float c[5]; std::vector<float> in[8], out[2];
    Run(const D* desc, int n) : d(desc) {
        h = d->inst(d, 48000); for (int k = 0; k < 8; ++k) in[k].assign(n, 0.f); for (int e = 0; e < 2; ++e) out[e].assign(n, 0.f);
        setYaw(0); c[4] = 1;
        for (int k = 0; k < 5; ++k) d->conn(h, 10 + k, &c[k]);
        if (d->act) d->act(h);
    }
    void setQuat(double w, double x, double y, double z) { c[0] = (float)w; c[1] = (float)x; c[2] = (float)y; c[3] = (float)z; }
    void setYaw(double rad) { setQuat(cos(rad / 2), 0, 0, sin(rad / 2)); }          // head turned by rad about the vertical (the speakers move by +rad)
    void setPitch(double rad) { setQuat(cos(rad / 2), 0, sin(rad / 2), 0); }
    // process [from, to) in 480-sample blocks, calling pose(sample) before each block
    template <class F> void go(int from, int to, F pose) {
        for (int p = from; p < to; p += 480) {
            pose(p); for (int k = 0; k < 8; ++k) d->conn(h, k, in[k].data() + p); for (int e = 0; e < 2; ++e) d->conn(h, 8 + e, out[e].data() + p);
            d->run(h, 480);
        }
    }
};
static double rms(const std::vector<float>& y, size_t a, size_t b) { double s = 0; for (size_t i = a; i < b; ++i) s += (double)y[i] * y[i]; return std::sqrt(s / (b - a)); }

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: dsp <libspatial-dsp.so>\n"); return 2; }
    void* lib = dlopen(argv[1], RTLD_NOW); if (!lib) { puts(dlerror()); return 1; }
    auto desc = (const D* (*)(unsigned long))dlsym(lib, "ladspa_descriptor");
    const D* amb = desc(0); check(amb && amb->pc == 15, "ambirot descriptor", "15 ports");
    if (!amb) return 1;
    const int N = 48000 * 2; char msg[160];

    // Mix = 0 is plain stereo (this is what the fade-out ends in)
    { Run r(amb, N); for (int i = 0; i < N; ++i) { r.in[0][i] = 0.3f * sinf(2 * M_PI * 1000 * i / 48000.f); r.in[1][i] = 0.2f * sinf(2 * M_PI * 440 * i / 48000.f); }
      r.c[4] = 0; r.go(0, N, [](int) {}); double e = 0; for (int i = 48000; i < N; ++i) e = std::fmax(e, std::fabs(r.out[0][i] - r.in[0][i]) + std::fabs(r.out[1][i] - r.in[1][i]));
      snprintf(msg, sizeof msg, "largest difference %.2e", e); check(e < 1e-5, "Mix 0: output equals input", msg); }

    // Is there rendering data? (rendered output differs from the input)
    bool haveData; { Run r(amb, 9600); for (int i = 0; i < 9600; ++i) r.in[0][i] = r.in[1][i] = 0.3f * sinf(2 * M_PI * 1000 * i / 48000.f); r.go(0, 9600, [](int) {});
      double e = 0; for (int i = 4800; i < 9600; ++i) e += std::fabs(r.out[0][i] - r.in[0][i]); haveData = e > 1.0; }
    if (!haveData) { puts("rendering checks skipped (run `make hrtf` first)"); return failures ? 1 : 0; }

    // Centred sound keeps the original level (no loudness boost): pink-ish noise, level of the output vs the input
    { Run r(amb, N); srand(3); float lp = 0; for (int i = 0; i < N; ++i) { float x = rand() / (float)RAND_MAX - 0.5f; lp += 0.3f * (x - lp); r.in[0][i] = r.in[1][i] = 0.2f * (x + 3 * lp); }
      r.go(0, N, [](int) {}); double g = 20 * log10(0.5 * (rms(r.out[0], 24000, N) + rms(r.out[1], 24000, N)) / rms(r.in[0], 24000, N));
      snprintf(msg, sizeof msg, "%.1f dB", g); check(std::fabs(g) < 3.0, "centred level within 3 dB of the input", msg); }

    // Turning never exceeds full scale (limiter): loud high-passed noise, head turned towards one speaker
    { Run r(amb, N); srand(5); float a = 0; for (int i = 0; i < N; ++i) { float x = (rand() / (float)RAND_MAX - 0.5f) * 1.8f; r.in[0][i] = r.in[1][i] = x - a; a = x; }
      r.setYaw(60 * M_PI / 180);
      r.go(0, N, [](int) {}); double pk = 0; for (int e = 0; e < 2; ++e) for (int i = 0; i < N; ++i) pk = std::fmax(pk, std::fabs(r.out[e][i]));
      snprintf(msg, sizeof msg, "peak %.3f", pk); check(pk <= 1.0, "output never exceeds full scale", msg); }

    // Smooth when the speakers pass straight over the head (the pole): a 1 kHz tone, the direction sweeps through the top
    { Run r(amb, N); for (int i = 0; i < N; ++i) r.in[0][i] = r.in[1][i] = 0.2f * sinf(2 * M_PI * 1000 * i / 48000.f);
      r.go(0, N, [&](int p) { r.setPitch(M_PI * ((double)p / N - 0.5) * 1.6); });      // nod -144 .. +144 degrees: the speakers pass over/under the head
      // residual after fitting a 1 kHz sinusoid per 10 ms window: clicks / steps show up here
      double worst = -200; for (int e = 0; e < 2; ++e) for (size_t s0 = 12000; s0 + 480 < (size_t)N; s0 += 480) {
          double a = 0, b = 0; for (int k = 0; k < 480; ++k) { a += r.out[e][s0 + k] * cos(2 * M_PI * 1000 * k / 48000); b += r.out[e][s0 + k] * sin(2 * M_PI * 1000 * k / 48000); }
          a *= 2.0 / 480; b *= 2.0 / 480; double res = 0, sig = 0;
          for (int k = 0; k < 480; ++k) { double x = r.out[e][s0 + k] - a * cos(2 * M_PI * 1000 * k / 48000) - b * sin(2 * M_PI * 1000 * k / 48000); res += x * x; sig += (double)r.out[e][s0 + k] * r.out[e][s0 + k]; }
          if (sig > 1e-4) worst = std::fmax(worst, 10 * log10(res / sig + 1e-12)); }
      snprintf(msg, sizeof msg, "worst window %.1f dB below the tone", -worst); check(worst < -30, "no clicks while the speakers cross the pole", msg); }

    // The direction of the speakers matters at all: turning the head changes the two ears differently
    { Run r(amb, 9600); for (int i = 0; i < 9600; ++i) r.in[0][i] = r.in[1][i] = 0.3f * sinf(2 * M_PI * 3000 * i / 48000.f);
      r.setYaw(90 * M_PI / 180);
      r.go(0, 9600, [](int) {}); double l = rms(r.out[0], 4800, 9600), rr = rms(r.out[1], 4800, 9600);
      snprintf(msg, sizeof msg, "ears differ by %.1f dB", 20 * log10(l / rr)); check(std::fabs(20 * log10(l / rr)) > 3.0, "head turn changes the two ears", msg); }

    // Surround: each channel is a virtual speaker in its own place (3 kHz tone in one channel, head facing forward)
    auto only = [&](int ch, double freq, int len, float amp, bool mixOn) { Run r(amb, len); for (int i = 0; i < len; ++i) r.in[ch][i] = amp * sinf(2 * M_PI * freq * i / 48000.f); r.c[4] = mixOn ? 1.f : 0.f; r.go(0, len, [](int) {});
        return std::pair<double, double>(rms(r.out[0], len / 2, len), rms(r.out[1], len / 2, len)); };
    { auto p = only(2, 3000, 14400, 0.3f, true); double d = 20 * log10(p.first / p.second); snprintf(msg, sizeof msg, "L-R %.1f dB", d); check(std::fabs(d) < 1.5, "centre channel sits in the middle", msg); }
    { auto p = only(6, 3000, 14400, 0.3f, true); double d = 20 * log10(p.first / p.second); snprintf(msg, sizeof msg, "L-R %.1f dB", d); check(d > 3.0, "left side channel is louder in the left ear", msg); }
    { auto p = only(7, 3000, 14400, 0.3f, true); double d = 20 * log10(p.second / p.first); snprintf(msg, sizeof msg, "R-L %.1f dB", d); check(d > 3.0, "right side channel is louder in the right ear", msg); }
    { auto p = only(4, 3000, 14400, 0.3f, true); double d = 20 * log10(p.first / p.second); snprintf(msg, sizeof msg, "L-R %.1f dB", d); check(d > 2.0, "rear left channel favours the left ear", msg); }
    { auto p = only(3, 80, 14400, 0.4f, true); double d = 20 * log10(p.first / p.second); snprintf(msg, sizeof msg, "L-R %.2f dB, level %.3f", d, p.first); check(std::fabs(d) < 0.3 && p.first > 0.05, "LFE reaches both ears equally", msg); }
    { auto p = only(2, 1000, 14400, 0.3f, false); snprintf(msg, sizeof msg, "%.3f / %.3f", p.first, p.second); check(std::fabs(p.first - p.second) < 1e-4 && std::fabs(p.first - 0.3 * 0.7071 / 1.41421356) < 0.01, "Mix 0: centre folds into both sides at -3 dB", msg); }
    return failures ? 1 : 0;
}
