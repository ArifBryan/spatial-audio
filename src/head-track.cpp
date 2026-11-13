// head-track: drives the Ambisonics spatial filter from OpenTrack-style UDP packets (SonyTrackerLinux).
// Event driven: sleeps in poll() until a head-tracking packet or a signal arrives.
//
//   packet   -> head pose -> where the two virtual speakers are in the head's frame -> plugin controls (PipeWire Props, no extra process)
//   SIGUSR2 -> if the switch is off: fade the effect out and exit
//   SIGHUP   -> sensor went silent: speakers return to the front, re-centre when data returns        SIGTERM -> plain stereo, exit
//
// Build: make        Usage: head-track --node ID [--port 4242]

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <pipewire/pipewire.h>
#include <spa/pod/builder.h>
#include <spa/param/props.h>


static const double PI = 3.14159265358979323846;
static inline double rad(double d) { return d * PI / 180.0; }
static inline double deg(double r) { return r * 180.0 / PI; }
static inline double wrap(double a) { a = std::fmod(a + 180.0, 360.0); if (a < 0) a += 360.0; return a - 180.0; }
static double nowSecs() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ------------------------------------------------------------------------------------------ options
struct Options {
    int node = -1, port = 4242;
    bool invert = false, invertPitch = false, invertRoll = false;
    double stillSecs = 2.0, stillDps = 6.0, settleSecs = 1.2;
} opt;

static std::string home() { const char* h = getenv("HOME"); return h ? h : "."; }
static const std::string FLAG = home() + "/.local/state/spatial-audio/enabled";
static const std::string STRENGTH_FILE = home() + "/.config/spatial-audio/strength";
static const std::string LOOKAHEAD_FILE = home() + "/.config/spatial-audio/lookahead";
static const std::string STATUS_FILE = std::string(getenv("XDG_RUNTIME_DIR") ? getenv("XDG_RUNTIME_DIR") : "/tmp") + "/spatial-audio/pose";   // what the helper is doing right now (for `spatial doctor`)

// A number read from a small settings file, re-read only when the file changes.
struct FileValue {
    std::string path; double value, lo, hi; time_t mtime = 0; long nsec = -1;
    FileValue(std::string p, double def, double l, double h) : path(std::move(p)), value(def), lo(l), hi(h) {}
    double get() {
        struct stat st;
        if (stat(path.c_str(), &st) != 0) return value;
        if (st.st_mtime != mtime || st.st_mtim.tv_nsec != nsec) {
            mtime = st.st_mtime; nsec = st.st_mtim.tv_nsec;
            std::ifstream f(path); double v;
            if (f >> v) value = std::min(hi, std::max(lo, v));
        }
        return value;
    }
};

// ------------------------------------------------------------------------------------------ PipeWire link
// Direct connection to the filter node: settings are sent as Props "params" with no extra process in between.
struct PwNode {
    pw_main_loop* ml = nullptr; pw_loop* loop = nullptr; pw_context* ctx = nullptr; pw_core* core = nullptr;
    pw_registry* reg = nullptr; pw_node* node = nullptr; spa_hook regHook{}, nodeHook{};
    uint32_t id = 0; bool bound = false, gone = false;

    static void onGlobal(void* d, uint32_t gid, uint32_t, const char* type, uint32_t, const spa_dict*) {
        auto* self = (PwNode*)d;
        if (gid != self->id || strcmp(type, PW_TYPE_INTERFACE_Node) != 0 || self->node) return;
        self->node = (pw_node*)pw_registry_bind(self->reg, gid, PW_TYPE_INTERFACE_Node, PW_VERSION_NODE, 0);
        self->bound = self->node != nullptr;
    }
    static void onRemove(void* d, uint32_t gid) { auto* self = (PwNode*)d; if (gid == self->id) self->gone = true; }

    bool connect(uint32_t nodeId) {
        id = nodeId;
        pw_init(nullptr, nullptr);
        ml = pw_main_loop_new(nullptr); loop = pw_main_loop_get_loop(ml);
        ctx = pw_context_new(loop, nullptr, 0);
        core = ctx ? pw_context_connect(ctx, nullptr, 0) : nullptr;
        if (!core) return false;
        reg = pw_core_get_registry(core, PW_VERSION_REGISTRY, 0);
        static const pw_registry_events ev = {PW_VERSION_REGISTRY_EVENTS, onGlobal, onRemove};
        pw_registry_add_listener(reg, &regHook, &ev, this);
        pw_loop_enter(loop);
        for (int i = 0; i < 60 && !bound; ++i) pw_loop_iterate(loop, 50);     // wait (briefly) for the node to be announced
        return bound;
    }
    int fd() { return pw_loop_get_fd(loop); }
    void pump() { pw_loop_iterate(loop, 0); }                                 // let PipeWire flush and deliver events
    void setParams(const std::vector<std::pair<std::string, float>>& kv) {
        if (!node) return;
        uint8_t buf[16384]; spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof buf); spa_pod_frame f[2];
        spa_pod_builder_push_object(&b, &f[0], SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
        spa_pod_builder_prop(&b, SPA_PROP_params, 0);
        spa_pod_builder_push_struct(&b, &f[1]);
        for (const auto& p : kv) { spa_pod_builder_string(&b, p.first.c_str()); spa_pod_builder_float(&b, p.second); }
        spa_pod_builder_pop(&b, &f[1]);
        auto* pod = (spa_pod*)spa_pod_builder_pop(&b, &f[0]);
        pw_node_set_param(node, SPA_PARAM_Props, 0, pod);
        pump();
    }
    void stop() {
        if (node) { pw_proxy_destroy((pw_proxy*)node); node = nullptr; }
        if (reg) { pw_proxy_destroy((pw_proxy*)reg); reg = nullptr; }
        if (core) { pw_core_disconnect(core); core = nullptr; }
        if (ctx) { pw_context_destroy(ctx); ctx = nullptr; }
        if (ml) { pw_main_loop_destroy(ml); ml = nullptr; }
    }
} pw;

struct ParamList {                                  // (name, value) pairs for one Props update
    std::vector<std::pair<std::string, float>> kv;
    void add(const std::string& name, double v) { kv.emplace_back(name, (float)v); }
};
static void push(const ParamList& p) { pw.setParams(p.kv); }

// ------------------------------------------------------------------------------------------ stillness recentering
struct Recenterer {
    double stillSecs, stillDps, settleSecs;
    double ref = 0, rate = 0, prevYaw = 0, prevT = 0, stillSince = 0; bool havePrev = false, still = false;
    void reset(double yaw) { ref = yaw; rate = 0; havePrev = false; still = false; }
    double update(double yaw, double now) {
        if (havePrev && now > prevT) {
            double dt = now - prevT, inst = std::fabs(wrap(yaw - prevYaw)) / dt;
            rate += std::min(dt / 0.3, 1.0) * (inst - rate);
            if (rate < stillDps) {
                if (!still) { still = true; stillSince = prevT; }
                if (now - stillSince >= stillSecs) ref += wrap(yaw - ref) * std::min(dt / settleSecs, 1.0);
            } else still = false;
        }
        prevYaw = yaw; prevT = now; havePrev = true;
        return ref;
    }
};

// ------------------------------------------------------------------------------------------ rotations
// The head orientation is kept as a rotation (quaternion) and smoothed / predicted as a rotation. Smoothing the three
// Euler angles separately breaks near "looking straight down/up", where yaw and roll swing by up to 180 degrees each
// while the real orientation changes smoothly.
struct Vec { double x, y, z; };
struct Mat { double m[3][3]; };
static Mat mmul(const Mat& a, const Mat& b) { Mat r{}; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j]; return r; }
static Mat rotZ(double d) { double c = std::cos(rad(d)), s = std::sin(rad(d)); return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }
static Mat rotY(double d) { double c = std::cos(rad(d)), s = std::sin(rad(d)); return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
static Mat rotX(double d) { double c = std::cos(rad(d)), s = std::sin(rad(d)); return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
// world -> head, the same convention as before: yaw about z, then pitch about y, then roll about x (degrees).
static Mat headMatrix(double yaw, double pitch, double roll) { return mmul(rotX(roll), mmul(rotY(pitch), rotZ(yaw))); }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    static Quat fromMat(const Mat& R) {
        Quat q; double t = R.m[0][0] + R.m[1][1] + R.m[2][2];
        if (t > 0) { double s = std::sqrt(t + 1) * 2; q.w = s / 4; q.x = (R.m[2][1] - R.m[1][2]) / s; q.y = (R.m[0][2] - R.m[2][0]) / s; q.z = (R.m[1][0] - R.m[0][1]) / s; }
        else if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) { double s = std::sqrt(1 + R.m[0][0] - R.m[1][1] - R.m[2][2]) * 2; q.w = (R.m[2][1] - R.m[1][2]) / s; q.x = s / 4; q.y = (R.m[0][1] + R.m[1][0]) / s; q.z = (R.m[0][2] + R.m[2][0]) / s; }
        else if (R.m[1][1] > R.m[2][2]) { double s = std::sqrt(1 + R.m[1][1] - R.m[0][0] - R.m[2][2]) * 2; q.w = (R.m[0][2] - R.m[2][0]) / s; q.x = (R.m[0][1] + R.m[1][0]) / s; q.y = s / 4; q.z = (R.m[1][2] + R.m[2][1]) / s; }
        else { double s = std::sqrt(1 + R.m[2][2] - R.m[0][0] - R.m[1][1]) * 2; q.w = (R.m[1][0] - R.m[0][1]) / s; q.x = (R.m[0][2] + R.m[2][0]) / s; q.y = (R.m[1][2] + R.m[2][1]) / s; q.z = s / 4; }
        return q.norm();
    }
    Quat norm() const { double n = std::sqrt(w * w + x * x + y * y + z * z); return {w / n, x / n, y / n, z / n}; }
    Quat conj() const { return {w, -x, -y, -z}; }
    Quat operator*(const Quat& b) const { return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y, w * b.y - x * b.z + y * b.w + z * b.x, w * b.z + x * b.y - y * b.x + z * b.w}; }
    Vec rotate(const Vec& v) const {                        // v' = q v q*
        Quat p{0, v.x, v.y, v.z}, r = (*this) * p * conj(); return {r.x, r.y, r.z};
    }
    double angleDeg() const { return deg(2 * std::acos(std::min(1.0, std::fabs(w)))); }              // rotation away from "facing forward"
};
static Quat slerp(Quat a, Quat b, double t) {
    double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; if (d < 0) { b = {-b.w, -b.x, -b.y, -b.z}; d = -d; }   // shortest way round
    if (d > 0.9995) return Quat{a.w + t * (b.w - a.w), a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)}.norm();
    double th = std::acos(d), s = std::sin(th), wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return Quat{wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z}.norm();
}

// Strength < 100: the sound stage follows only a share of the head rotation. Scaling a rotation is not continuous all the way
// round (at 180 degrees the axis flips sign), so the share fades to 100 % over the last 90 degrees before 180: the stage
// follows exactly when you are facing backwards, and there is no seam there.
static Quat scaleRotation(Quat q, double share) {
    if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
    double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z); if (s < 1e-9 || share >= 1.0) return q;
    double theta = 2 * std::atan2(s, q.w), u = std::min(1.0, std::max(0.0, (theta - rad(90.0)) / rad(90.0))), f = u * u * (3 - 2 * u);
    double t2 = theta * (share + (1.0 - share) * f), h = t2 / 2, k = std::sin(h) / s;
    return Quat{std::cos(h), q.x * k, q.y * k, q.z * k};
}

// ------------------------------------------------------------------------------------------ look-ahead (on the rotation)
struct Predictor {
    double lookahead = 0, maxLeadDeg = 15, deadbandDps = 4, tau = 0.12, prevT = 0; Vec w{0, 0, 0}; Quat prev; bool have = false;
    void reset() { w = {0, 0, 0}; have = false; }
    Quat update(const Quat& q, double now) {
        if (have && now > prevT) {
            double dt = now - prevT; Quat d = prev.conj() * q; if (d.w < 0) d = {-d.w, -d.x, -d.y, -d.z};
            double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), ang = 2 * std::atan2(s, d.w), k = s > 1e-9 ? ang / s / dt : 0;
            double a = std::min(dt / tau, 1.0); w = {w.x + a * (d.x * k - w.x), w.y + a * (d.y * k - w.y), w.z + a * (d.z * k - w.z)};
        }
        prev = q; prevT = now; have = true;
        double speed = deg(std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z));
        if (speed < deadbandDps || lookahead <= 0) return q;
        double lead = std::min(rad(maxLeadDeg), rad(speed) * lookahead), h = lead / 2, sp = std::sin(h) / (std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z));
        return (q * Quat{std::cos(h), w.x * sp, w.y * sp, w.z * sp}).norm();
    }
};
// ------------------------------------------------------------------------------------------ head model
static const double SPK_AZ[2] = {30.0, -30.0};       // virtual speakers: left, right (in the room, at ear height)
static Vec speakerDir(int s) { double a = rad(SPK_AZ[s]); return {std::cos(a), std::sin(a), 0.0}; }

// The plugin takes the speaker directions as unit vectors in the head frame (x forward, y left, z up) and fades to plain stereo with "Mix".
static void addDirections(ParamList& p, const Quat& q) {
    for (int ch = 0; ch < 2; ++ch) {
        Vec v = q.rotate(speakerDir(ch)); const std::string n = "ambi:";
        p.add(n + "X " + std::to_string(ch + 1), v.x); p.add(n + "Y " + std::to_string(ch + 1), v.y); p.add(n + "Z " + std::to_string(ch + 1), v.z);
    }
}

// ------------------------------------------------------------------------------------------ main
static void usage() { fprintf(stderr, "usage: head-track --node ID [--port N] [--invert] [--invert-pitch] [--invert-roll]\n"
    "       [--still-secs S] [--still-dps D] [--settle-secs S]\n"); exit(2); }

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i]; auto val = [&]() { if (i + 1 >= argc) usage(); return std::string(argv[++i]); };
        if (a == "--node") opt.node = atoi(val().c_str()); else if (a == "--port") opt.port = atoi(val().c_str());
        else if (a == "--invert") opt.invert = true; else if (a == "--invert-pitch") opt.invertPitch = true; else if (a == "--invert-roll") opt.invertRoll = true;
        else if (a == "--still-secs") opt.stillSecs = atof(val().c_str());
        else if (a == "--still-dps") opt.stillDps = atof(val().c_str()); else if (a == "--settle-secs") opt.settleSecs = atof(val().c_str());
        else usage();
    }
    if (opt.node < 0) usage();
    if (opt.stillSecs <= 0) opt.stillSecs = 1e12;
    const double SIGN = opt.invert ? -1.0 : 1.0;

    // Block the signals BEFORE PipeWire starts its threads (they inherit the mask); otherwise a thread without the mask takes the
    // signal's default action and kills the process instead of letting the loop fade out.
    sigset_t mask; sigemptyset(&mask);
    for (int s : {SIGUSR2, SIGHUP, SIGTERM, SIGINT}) sigaddset(&mask, s);
    sigprocmask(SIG_BLOCK, &mask, nullptr);
    int sfd = signalfd(-1, &mask, SFD_CLOEXEC);

    if (!pw.connect(opt.node)) { fprintf(stderr, "cannot reach PipeWire node %d\n", opt.node); return 1; }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(opt.port); inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    int one = 1; setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    if (bind(sock, (sockaddr*)&addr, sizeof addr) != 0) { perror("bind"); return 1; }


    // strength = how much of the head movement the sound stage follows (100 = exactly as you turn)
    FileValue strength(STRENGTH_FILE, 100.0, 0.0, 100.0), lookahead(LOOKAHEAD_FILE, 0.0, 0.0, 250.0);
    Recenterer rcY{opt.stillSecs, opt.stillDps, opt.settleSecs};
    Predictor pred; Quat qs; bool haveQs = false;
    bool silent = false, haveLast = false;
    double last[4] = {0, 0, 0, 0}, loggedBucket = 1e9;

    auto centre = [&]() { ParamList p; addDirections(p, Quat()); p.add("ambi:Mix", 1.0); push(p); haveLast = false; };   // speakers straight ahead

    printf("head-track listening on 127.0.0.1:%d for node %d\n", opt.port, opt.node); fflush(stdout);
    pollfd fds[3] = {{sock, POLLIN, 0}, {sfd, POLLIN, 0}, {pw.fd(), POLLIN, 0}};
    for (;;) {
        if (poll(fds, 3, -1) < 0) continue;          // signals are blocked and read through signalfd: no lost wake-ups
        if (fds[2].revents & POLLIN) { pw.pump(); if (pw.gone) { fprintf(stderr, "filter node went away\n"); pw.stop(); return 0; } }
        if (fds[1].revents & POLLIN) {
            signalfd_siginfo si; if (read(sfd, &si, sizeof si) == sizeof si) {
                switch (si.ssi_signo) {
                    case SIGHUP: haveQs = false; silent = true; centre(); break;
                    case SIGUSR2:
                        if (access(FLAG.c_str(), F_OK) != 0) {            // switched off: glide to plain stereo (the filter stays in the path)
                            { ParamList p; p.add("ambi:Mix", 0.0); push(p); usleep(250000); }   // the plugin glides to plain stereo over ~50 ms
                            pw.stop(); return 0;
                        }
                        break;
                    default: { ParamList p; p.add("ambi:Mix", 0.0); push(p); pw.stop(); return 0; }   // SIGTERM / SIGINT
                }
            }
        }
        if (!(fds[0].revents & POLLIN)) continue;
        double d[6]; ssize_t n = recv(sock, d, sizeof d, 0);
        if (n < (ssize_t)sizeof d || access(FLAG.c_str(), F_OK) != 0) continue;

        double yaw = wrap(d[3]) * SIGN, pitch = d[4] * (opt.invertPitch ? -1 : 1), roll = d[5] * (opt.invertRoll ? -1 : 1), now = nowSecs();
        if (silent) { rcY.reset(yaw); pred.reset(); haveQs = false; silent = false; }   // after a gap in the data, start from here
        // Stillness-based heading recenter (Android style). Looking steeply up/down the yaw angle means nothing (it can
        // swing by 180 degrees), so the centre is left alone there instead of learning from it.
        double ref = std::cos(rad(pitch)) > 0.3 ? rcY.update(yaw, now) : rcY.ref;
        // Recentre by turning the *heading* only: exactly M_raw * Rz(-ref), which stays smooth even where the three angles flip.
        Quat qc = Quat::fromMat(headMatrix(wrap(yaw - ref), pitch, roll));
        qs = haveQs ? slerp(qs, qc, 0.35) : qc; haveQs = true;                  // smooth as a rotation (shortest way, no gimbal flips)
        pred.lookahead = lookahead.get() / 1000.0;
        Quat aim = pred.update(qs, now);
        if (aim.w < 0) aim = {-aim.w, -aim.x, -aim.y, -aim.z};
        aim = scaleRotation(aim, strength.get() / 100.0);

        double cur[4] = {std::round(aim.w * 500) / 500, std::round(aim.x * 500) / 500, std::round(aim.y * 500) / 500, std::round(aim.z * 500) / 500};
        if (haveLast && !memcmp(cur, last, sizeof cur)) continue;                  // nothing audible changed
        memcpy(last, cur, sizeof cur); haveLast = true;

        ParamList p; addDirections(p, aim); p.add("ambi:Mix", 1.0);
        push(p);

        if (FILE* sf = fopen(STATUS_FILE.c_str(), "w")) {                               // overwritten on every applied update
            fprintf(sf, "turned=%.1f deg strength=%.0f recentre_ref=%.1f\n", aim.angleDeg(), strength.get(), rcY.ref);
            fclose(sf);
        }
        double bucket = std::floor(aim.angleDeg() / 5.0);
        if (bucket != loggedBucket) { loggedBucket = bucket; printf("turned %6.1f deg\n", aim.angleDeg()); fflush(stdout); }
    }
}
