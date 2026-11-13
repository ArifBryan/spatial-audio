// spatial-tools: offline tool that builds the Ambisonics renderer's data from a SOFA HRTF set.
//   spatial-tools gen-ambi [file.sofa]   fit the HRTF into order-3 spherical-harmonic channels and write
//                                        data/ambi/ambi.bin and build/head-tracked-ambi.conf (the PipeWire filter graph)
// Uses libmysofa (headers: libmysofa-dev). Any SOFA file with measured HRIRs works, so a personalised set is a drop-in.
#include "sh.h"
#include <mysofa.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

static const double PI = 3.14159265358979323846, SR = 48000.0;
static std::string HERE, ROOT, DATA;                   // HERE = build/ (binaries, generated graph), DATA = <repo>/data
typedef std::complex<double> Cx;

struct Sofa {
    MYSOFA_EASY* h = nullptr; int taps = 0;
    bool open(const std::string& path) {
        int len = 0, err = 0; h = mysofa_open(path.c_str(), (float)SR, &len, &err);
        taps = len; if (!h) fprintf(stderr, "cannot open '%s' (error %d)\n", path.c_str(), err); return h != nullptr;
    }
    // impulse responses (left ear, right ear) and delays for a source direction (degrees; + az = left, + el = up)
    void ir(double az, double el, std::vector<float>& L, std::vector<float>& R, float& dl, float& dr, double r = 1.5) const {
        L.assign(taps, 0); R.assign(taps, 0);
        double a = az * PI / 180, e = el * PI / 180;
        mysofa_getfilter_float(h, (float)(r * cos(e) * cos(a)), (float)(r * cos(e) * sin(a)), (float)(r * sin(e)), L.data(), R.data(), &dl, &dr);
    }
};
static std::complex<double> response(const std::vector<float>& x, double delay, double f) {
    std::complex<double> s = 0; double w = 2 * PI * f / SR;
    for (size_t i = 0; i < x.size(); ++i) s += (double)x[i] * std::polar(1.0, -w * (i + delay * SR));
    return s;
}
static void fft(std::vector<Cx>& a, bool inverse) {                 // iterative radix-2; a.size() must be a power of two
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) { size_t bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) std::swap(a[i], a[j]); }
    for (size_t len = 2; len <= n; len <<= 1) {
        Cx wl = std::polar(1.0, 2 * PI / len * (inverse ? 1 : -1));
        for (size_t i = 0; i < n; i += len) { Cx w = 1; for (size_t j = 0; j < len / 2; ++j) { Cx u = a[i + j], v = a[i + j + len / 2] * w; a[i + j] = u + v; a[i + j + len / 2] = u - v; w *= wl; } }
    }
    if (inverse) for (auto& x : a) x /= (double)n;
}
// Same magnitude response, but with all the bulk/arrival delay removed (homomorphic / cepstral method). Blending
// minimum-phase filters of neighbouring directions cannot cancel frequencies, unlike blending filters with different delays.
static std::vector<float> minimumPhase(const std::vector<float>& h, int taps) {
    const size_t M = 4096; std::vector<Cx> x(M, 0.0); for (size_t i = 0; i < h.size() && i < M; ++i) x[i] = h[i];
    fft(x, false);
    for (auto& v : x) v = std::log(std::max(std::abs(v), 1e-6));                      // log magnitude
    fft(x, true);                                                                      // real cepstrum
    std::vector<Cx> c(M, 0.0); c[0] = x[0];
    for (size_t i = 1; i < M / 2; ++i) c[i] = 2.0 * x[i].real();                      // fold: keep the causal part
    c[M / 2] = x[M / 2].real();
    fft(c, false); for (auto& v : c) v = std::exp(v);                                  // back to a spectrum (minimum phase)
    fft(c, true);
    std::vector<float> out(taps); for (int i = 0; i < taps; ++i) out[i] = (float)c[i].real();
    return out;
}
// Interaural time difference (seconds; + = right ear later = source on the left): the lag at which the two ears' low-passed
// impulse responses line up best (cross-correlation, sub-sample accurate). A phase-slope estimate wraps around for sources
// to the side, where the difference exceeds half a period of the frequencies used.
static double itdOf(const std::vector<float>& L, double dl, const std::vector<float>& R, double dr) {
    const size_t M = 1024; std::vector<Cx> a(M, 0.0), b(M, 0.0);
    for (size_t i = 0; i < L.size() && i < M; ++i) { a[i] = L[i]; b[i] = R[i]; }
    fft(a, false); fft(b, false);
    std::vector<Cx> x(M);
    for (size_t k = 0; k < M; ++k) {                                  // cross-spectrum, band-limited to ~1.5 kHz where timing dominates
        double f = (k <= M / 2 ? k : M - k) * SR / M, w = f < 1500 ? 0.5 * (1 + cos(PI * f / 1500)) : 0.0;
        x[k] = std::conj(a[k]) * b[k] * w;
    }
    fft(x, true);                                                      // x[lag] = how well R(t + lag) matches L(t)
    int best = 0; double bv = -1e30;
    for (int lag = -64; lag <= 64; ++lag) { double v = x[(lag + (int)M) % M].real(); if (v > bv) { bv = v; best = lag; } }
    double ym = x[(best - 1 + (int)M) % M].real(), y0 = x[(best + (int)M) % M].real(), yp = x[(best + 1 + (int)M) % M].real();
    double frac = (ym - 2 * y0 + yp) != 0 ? 0.5 * (ym - yp) / (ym - 2 * y0 + yp) : 0.0;        // parabolic peak
    return ((best + frac) / SR) + (dr - dl);
}

static bool solveSym(std::vector<double> A, std::vector<double>& X, int n, int m) {      // X (n x m) = A^-1 * X, Gauss-Jordan
    for (int i = 0; i < n; ++i) {
        int p = i; for (int r = i + 1; r < n; ++r) if (fabs(A[r * n + i]) > fabs(A[p * n + i])) p = r;
        if (fabs(A[p * n + i]) < 1e-14) return false;
        if (p != i) { for (int k = 0; k < n; ++k) std::swap(A[i * n + k], A[p * n + k]); for (int k = 0; k < m; ++k) std::swap(X[i * m + k], X[p * m + k]); }
        double d = A[i * n + i]; for (int k = 0; k < n; ++k) A[i * n + k] /= d; for (int k = 0; k < m; ++k) X[i * m + k] /= d;
        for (int r = 0; r < n; ++r) if (r != i) { double f = A[r * n + i]; if (f == 0) continue; for (int k = 0; k < n; ++k) A[r * n + k] -= f * A[i * n + k]; for (int k = 0; k < m; ++k) X[r * m + k] -= f * X[i * m + k]; }
    }
    return true;
}


// The PipeWire filter graph: input -> Ambisonics renderer -> output. Nothing else sits in the signal path.
static void writeGraph() {
    const std::string plugin = HERE + "/libspatial-dsp.so";
    std::ostringstream c; c.setf(std::ios::fixed);
    c << "# Head-tracked spatial audio (Ambisonics). Generated by spatial-tools; controlled live by head-track.\n"
         "context.properties = { log.level = 2 }\ncontext.spa-libs = {\n  audio.convert.* = audioconvert/libspa-audioconvert\n  support.*       = support/libspa-support\n}\n"
         "context.modules = [\n  { name = libpipewire-module-rt flags = [ ifexists nofail ] }\n  { name = libpipewire-module-protocol-native }\n"
         "  { name = libpipewire-module-client-node }\n  { name = libpipewire-module-client-device }\n  { name = libpipewire-module-adapter }\n  { name = libpipewire-module-metadata }\n"
         "  { name = libpipewire-module-filter-chain\n    flags = [ nofail ]\n    args = {\n      node.description = \"Spatial Head-Tracked\"\n      media.name = \"Spatial Head-Tracked\"\n"
         "      filter.graph = {\n        nodes = [\n          { type = builtin label = copy name = cpL }\n          { type = builtin label = copy name = cpR }\n";
    c << "          { type = ladspa name = ambi plugin = \"" << plugin << "\" label = ambirot control = { \"X 1\" = " << cos(PI / 6) << " \"Y 1\" = 0.5 \"Z 1\" = 0.0 \"X 2\" = " << cos(PI / 6) << " \"Y 2\" = -0.5 \"Z 2\" = 0.0 \"Mix\" = 1.0 } }\n";
    c << "        ]\n        links = [\n          { output = \"cpL:Out\" input = \"ambi:In 1\" } { output = \"cpR:Out\" input = \"ambi:In 2\" }\n"
         "        ]\n        inputs  = [ \"cpL:In\" \"cpR:In\" ]\n        outputs = [ \"ambi:Out 1\" \"ambi:Out 2\" ]\n      }\n"
         "      capture.props = {\n        node.name = \"spatial_test_in\"\n        media.class = Audio/Sink\n        filter.smart = true\n        filter.smart.name = \"spatial-head-tracked\"\n"
         "        filter.smart.target = { bluez5.sink-loopback-target = true }\n        node.link-group = spatial-hp\n        audio.channels = 2\n        audio.position = [ FL FR ]\n"
         "        priority.session = 0\n        priority.driver = 0\n      }\n      playback.props = {\n        node.name = \"spatial_test_out\"\n        node.passive = true\n"
         "        node.link-group = spatial-hp\n        audio.channels = 2\n        audio.position = [ FL FR ]\n      }\n    }\n  }\n]\n";
    std::ofstream(HERE + "/head-tracked-ambi.conf") << c.str();
}

static int cmdGenAmbi(const std::string& sofa) {
    Sofa s; if (!s.open(sofa)) return 1;
    MYSOFA_HRTF* H = s.h->hrtf; const unsigned M = H->M, R = H->R, N = H->N; const int T = 128, C = SH_CHANNELS; const double SCALE = 1.0;
    std::vector<double> Y((size_t)M * C), itd(M); std::vector<float> hm((size_t)M * 2 * T);
    for (unsigned m = 0; m < M; ++m) {
        double x = H->SourcePosition.values[3 * m], y = H->SourcePosition.values[3 * m + 1], z = H->SourcePosition.values[3 * m + 2], nn = sqrt(x * x + y * y + z * z);
        shEval(x / nn, y / nn, z / nn, &Y[(size_t)m * C]);
        std::vector<float> L(H->DataIR.values + ((size_t)m * R + 0) * N, H->DataIR.values + ((size_t)m * R + 0) * N + N), Rr(H->DataIR.values + ((size_t)m * R + 1) * N, H->DataIR.values + ((size_t)m * R + 1) * N + N);
        double dl = 0, dr = 0; if (H->DataDelay.elements >= M * R) { dl = H->DataDelay.values[m * R]; dr = H->DataDelay.values[m * R + 1]; } else if (H->DataDelay.elements >= R) { dl = H->DataDelay.values[0]; dr = H->DataDelay.values[1]; }
        itd[m] = itdOf(L, dl / SR, Rr, dr / SR);
        std::vector<float> mp[2] = {minimumPhase(L, T), minimumPhase(Rr, T)};
        for (int e = 0; e < 2; ++e) for (int k = 0; k < T; ++k) hm[((size_t)m * 2 + e) * T + k] = mp[e][k] * (float)SCALE;
    }
    // normal equations (with a little regularisation) and their solution for every ear/tap and for the time difference
    std::vector<double> A((size_t)C * C, 0.0);
    for (unsigned m = 0; m < M; ++m) for (int i = 0; i < C; ++i) for (int j = 0; j < C; ++j) A[i * C + j] += Y[(size_t)m * C + i] * Y[(size_t)m * C + j];
    std::vector<double> A0 = A; double tr = 0; for (int i = 0; i < C; ++i) tr += A[i * C + i]; for (int i = 0; i < C; ++i) A[i * C + i] += 1e-4 * tr / C;
    const int cols = 2 * T + 1; std::vector<double> B((size_t)C * cols, 0.0);
    for (unsigned m = 0; m < M; ++m) for (int c = 0; c < C; ++c) {
        double y = Y[(size_t)m * C + c]; for (int k = 0; k < 2 * T; ++k) B[(size_t)c * cols + k] += y * hm[(size_t)m * 2 * T + k]; B[(size_t)c * cols + 2 * T] += y * itd[m];
    }
    if (!solveSym(A, B, C, cols)) { fprintf(stderr, "SH fit failed (singular)\n"); return 1; }
    std::vector<float> h((size_t)C * 2 * T), itdSh(C);
    for (int c = 0; c < C; ++c) { itdSh[c] = (float)B[(size_t)c * cols + 2 * T]; for (int e = 0; e < 2; ++e) for (int k = 0; k < T; ++k) h[((size_t)c * 2 + e) * T + k] = (float)B[(size_t)c * cols + e * T + k]; }

    // fit quality: relative error of the synthesised filters against the measured ones (all directions, both ears)
    double errNum = 0, errDen = 0;
    for (unsigned m = 0; m < M; m += 7) for (int e = 0; e < 2; ++e) for (int k = 0; k < T; ++k) {
        double v = 0; for (int c = 0; c < C; ++c) v += Y[(size_t)m * C + c] * h[((size_t)c * 2 + e) * T + k]; double t = hm[((size_t)m * 2 + e) * T + k]; errNum += (v - t) * (v - t); errDen += t * t; }
    printf("order-%d fit: relative error of the filters %.1f dB (0 dB would mean nothing captured)\n", SH_ORDER, 10 * log10(errNum / errDen));

    // diffuse-field correction: the order-limited fit loses high-frequency energy; restore it per frequency
    const int NF = 96; std::vector<double> fq(NF); for (int k = 0; k < NF; ++k) fq[k] = 100 * pow(190.0, k / (double)(NF - 1));    // 100 Hz .. 19 kHz
    std::vector<double> gcorr[2];
    for (int e = 0; e < 2; ++e) {
        std::vector<double> g(NF, 1.0);
        for (int k = 0; k < NF; ++k) {
            std::vector<Cx> Hc(C); for (int c = 0; c < C; ++c) { std::vector<float> hc(T); for (int i = 0; i < T; ++i) hc[i] = h[((size_t)c * 2 + e) * T + i]; Hc[c] = response(hc, 0, fq[k]); }
            double ps = 0; for (int i = 0; i < C; ++i) for (int j = 0; j < C; ++j) ps += (std::conj(Hc[i]) * A0[i * C + j] * Hc[j]).real(); ps /= M;
            double pt = 0; for (unsigned m = 0; m < M; m += 5) { std::vector<float> hh(hm.begin() + ((size_t)m * 2 + e) * T, hm.begin() + ((size_t)m * 2 + e) * T + T); pt += std::norm(response(hh, 0, fq[k])); } pt /= ((M + 4) / 5);
            g[k] = std::min(3.0, std::max(0.5, sqrt(pt / std::max(ps, 1e-20))));
        }
        std::vector<double> gs(NF); for (int k = 0; k < NF; ++k) { double sacc = 0; int cnt = 0; for (int j = std::max(0, k - 3); j <= std::min(NF - 1, k + 3); ++j) { sacc += g[j]; ++cnt; } gs[k] = sacc / cnt; }
        gcorr[e] = gs;
    }
    // Centre-matched level and tone. Music is partly centred (the two speakers add coherently) and partly wide (they add in
    // power); ALPHA is the share assumed coherent. The rendered response of the +-30 degree pair, smoothed over about a third
    // of an octave, is divided out, so that a centred sound has the level and tone of the original stereo and nothing is
    // louder than the input. Off-centre the response then varies naturally with direction, as a real HRTF does.
    const double ALPHA = 0.5;
    for (int e = 0; e < 2; ++e) {
        std::vector<double> P(NF);
        for (int k = 0; k < NF; ++k) {
            double pm_re = 0, pm_im = 0, pu = 0; Cx sum = 0;
            for (int sp = 0; sp < 2; ++sp) {
                double a = (sp ? -30.0 : 30.0) * PI / 180, Ys[SH_CHANNELS]; shEval(cos(a), sin(a), 0, Ys);
                Cx hs = 0; double itd = 0;
                for (int c = 0; c < C; ++c) { std::vector<float> hc(T); for (int i = 0; i < T; ++i) hc[i] = h[((size_t)c * 2 + e) * T + i]; hs += Ys[c] * response(hc, 0, fq[k]); itd += Ys[c] * itdSh[c]; }
                hs *= gcorr[e][k];
                double d = e == 0 ? std::max(0.0, -itd) : std::max(0.0, itd);
                sum += hs * std::polar(1.0, -2 * PI * fq[k] * d); pu += std::norm(hs);
            }
            (void)pm_re; (void)pm_im; P[k] = ALPHA * std::norm(sum) + (1 - ALPHA) * pu;
        }
        for (int k = 0; k < NF; ++k) {
            double acc = 0; int cnt = 0; for (int j = std::max(0, k - 4); j <= std::min(NF - 1, k + 4); ++j) { acc += P[j]; ++cnt; }
            gcorr[e][k] *= std::min(4.0, std::max(0.25, 1.0 / sqrt(std::max(acc / cnt, 1e-12))));      // limited to -12 .. +12 dB
        }
    }
    double gmin = 9, gmax = 0; for (int e = 0; e < 2; ++e) for (double g : gcorr[e]) { gmin = std::min(gmin, g); gmax = std::max(gmax, g); }
    printf("diffuse-field + centre correction range: %.1f .. %.1f dB\n", 20 * log10(gmin), 20 * log10(gmax));
    const size_t FFT = 1024;
    for (int c = 0; c < C; ++c) for (int e = 0; e < 2; ++e) {
        std::vector<Cx> X(FFT, 0.0); for (int i = 0; i < T; ++i) X[i] = h[((size_t)c * 2 + e) * T + i]; fft(X, false);
        for (size_t k = 0; k < FFT; ++k) {
            double f = (k <= FFT / 2 ? k : FFT - k) * SR / FFT, pos = std::log(std::max(f, 100.0) / 100.0) / std::log(190.0) * (NF - 1);
            int i0 = std::max(0, std::min(NF - 2, (int)pos)); double u = std::min(1.0, std::max(0.0, pos - i0)), g = gcorr[e][i0] * (1 - u) + gcorr[e][i0 + 1] * u; X[k] *= g;
        }
        fft(X, true); for (int i = 0; i < T; ++i) { double v = X[i].real(); if (i >= T - 24) v *= 0.5 * (1 + cos(PI * (i - (T - 24)) / 24)); h[((size_t)c * 2 + e) * T + i] = (float)v; }
    }
    std::string dir = DATA + "/ambi"; mkdir(DATA.c_str(), 0755); mkdir(dir.c_str(), 0755);
    FILE* f = fopen((dir + "/ambi.bin").c_str(), "wb"); uint32_t hdr[2] = {(uint32_t)C, (uint32_t)T};
    fwrite("SPAM1\0\0\0", 1, 8, f); fwrite(hdr, 4, 2, f); fwrite(itdSh.data(), 4, itdSh.size(), f); fwrite(h.data(), 4, h.size(), f); fclose(f);
    printf("exported %d Ambisonic channels x 2 ears x %d taps\n", C, T);
    writeGraph();
    printf("wrote head-tracked-ambi.conf\n");
    return 0;
}

// ------------------------------------------------------------------------------------------ bank export

int main(int argc, char** argv) {
    char buf[4096]; ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1); buf[n > 0 ? n : 0] = 0; HERE = buf; HERE = HERE.substr(0, HERE.rfind('/')); ROOT = HERE + "/.."; DATA = ROOT + "/data";
    if (argc < 2 || std::string(argv[1]) != "gen-ambi") { fprintf(stderr, "usage: spatial-tools gen-ambi [file.sofa]\n"); return 2; }
    std::string sofa = argc > 2 ? argv[2] : DATA + "/hrtf/D1_HRIR_SOFA/D1_48K_24bit_256tap_FIR_SOFA.sofa";     // SADIE II KU100, see `make hrtf`
    return cmdGenAmbi(sofa);
}
