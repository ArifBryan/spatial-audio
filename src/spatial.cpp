// spatial — command line for head-tracked spatial audio on Sony earbuds.
//   spatial on | off | toggle | status | query | strength [N] | lookahead [MS] | doctor
// The daemon (spatial-daemon, a systemd user service) does the work and reacts to the files this tool writes.
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

static std::string env(const char* k, const char* d) { const char* v = getenv(k); return v ? v : d; }
static const std::string HOME = env("HOME", ".");
static const std::string RUN = env("XDG_RUNTIME_DIR", "/tmp") + "/spatial-audio";
static const std::string FLAG = HOME + "/.local/state/spatial-audio/enabled";
static const std::string CONF = HOME + "/.config/spatial-audio";
static const char* UNIT = "spatial-audio.service";

static bool exists(const std::string& p) { return access(p.c_str(), F_OK) == 0; }
static std::string readFile(const std::string& p) { std::ifstream f(p); std::stringstream s; s << f.rdbuf(); return s.str(); }
static std::string trim(std::string s) { while (!s.empty() && isspace((unsigned char)s.back())) s.pop_back(); return s; }
static void mkdirs(const std::string& p) { std::string cur; std::stringstream ss(p); std::string part; while (std::getline(ss, part, '/')) { cur += (cur.empty() && p[0] != '/' ? "" : "/") + part; if (!part.empty()) mkdir(cur.c_str(), 0755); } }
static int sh(const std::string& cmd) { return system(cmd.c_str()); }

// The daemon writes {"enabled": b, "compatible": b, "active": b}; no JSON library needed for three booleans.
static bool stateFlag(const std::string& key, bool& found) {
    std::string s = readFile(RUN + "/state.json"); size_t p = s.find("\"" + key + "\"");
    found = p != std::string::npos; if (!found) return false;
    p = s.find(':', p); return s.compare(s.find_first_not_of(' ', p + 1), 4, "true") == 0;
}
static bool pidAlive(const std::string& name) {
    std::string p = trim(readFile(RUN + "/" + name + ".pid")); if (p.empty()) return false;
    return kill(atoi(p.c_str()), 0) == 0;
}
static std::string setting(const std::string& file, const char* def) { std::string v = trim(readFile(CONF + "/" + file)); return v.empty() ? def : v; }
static void writeSetting(const std::string& file, const std::string& v) { mkdirs(CONF); std::ofstream(CONF + "/" + file) << v << "\n"; }

static void query(std::string& on, std::string& compat) {
    bool found; bool en = stateFlag("enabled", found); on = found ? (en ? "on" : "off") : (exists(FLAG) ? "on" : "off");
    bool f2; bool c = stateFlag("compatible", f2); compat = c ? "compatible" : "absent";
}

int main(int argc, char** argv) {
    std::string cmd = argc > 1 ? argv[1] : "toggle", arg = argc > 2 ? argv[2] : "";
    mkdirs(HOME + "/.local/state/spatial-audio");
    if (cmd == "on") {
        FILE* f = fopen(FLAG.c_str(), "w"); if (f) fclose(f);
        sh(std::string("systemctl --user daemon-reload; systemctl --user enable ") + UNIT + " >/dev/null 2>&1; systemctl --user start " + UNIT);
        std::string on, c; query(on, c); puts(c == "compatible" ? "Spatial ON" : "Spatial ON (waiting: select stereo Sony earbuds as the output)");
    } else if (cmd == "off") {
        unlink(FLAG.c_str()); sh(std::string("systemctl --user start ") + UNIT + " 2>/dev/null"); puts("Spatial OFF");   // the daemon reacts to the file
    } else if (cmd == "toggle") {
        return execl("/proc/self/exe", argv[0], exists(FLAG) ? "off" : "on", (char*)nullptr);
    } else if (cmd == "query") {
        std::string on, c; query(on, c); printf("%s %s\n", on.c_str(), c.c_str());
    } else if (cmd == "status") {
        std::string on, c; query(on, c); bool f, act = stateFlag("active", f);
        if (on == "off") puts("Spatial: OFF");
        else if (c != "compatible") puts("Spatial: ON  (waiting for compatible earbuds as the output)");
        else printf("Spatial: ON  (%s, tracker: %s)\n", act ? "active" : "starting", pidAlive("tr") ? "running" : "waiting for sensor");
    } else if (cmd == "strength" || cmd == "lookahead") {
        bool s = cmd == "strength";
        if (arg.empty()) printf("%s: %s%s\n", cmd.c_str(), setting(cmd, s ? "100" : "0").c_str(), s ? "  (0-100)" : " ms  (0 = off)");
        else { writeSetting(cmd, arg); printf("%s set to %s (applies immediately)\n", cmd.c_str(), arg.c_str()); }
    } else if (cmd == "force") {                                  // TESTING: skip the hardware checks (see spatial-daemon)
        std::string f = CONF + "/force";
        if (arg == "on") { mkdirs(CONF); std::ofstream(f) << "1\n"; puts("force ON: hardware checks disabled (testing). The filter attaches to the selected output."); }
        else if (arg == "off") { unlink(f.c_str()); puts("force OFF: normal hardware checks"); }
        else printf("force: %s   (spatial force on|off)\n", exists(f) ? "ON (hardware checks disabled)" : "off");
    } else if (cmd == "doctor") {
        puts("== spatial doctor =="); fflush(stdout); sh("/proc/" + std::to_string(getpid()) + "/exe status"); sh("echo \"state: $(cat " + RUN + "/state.json 2>/dev/null)\"");
        fflush(stdout); puts("  helper right now (turned angle, spatial share, ...):"); fflush(stdout); sh("cat " + RUN + "/pose 2>/dev/null | sed 's/^/   /'"); fflush(stdout); puts("  processes:"); fflush(stdout); sh("pgrep -a -f '^(pipewire -c .*head-tracked|.*/head-track --node|.*/spatial-daemon$|.*/sony-tracker)' | cut -c1-110 | sed 's/^/   /'");
        fflush(stdout); puts("  filter links:"); fflush(stdout); sh("pw-link -l 2>/dev/null | awk '/^spatial_test_in:playback/{p=1;next} /^[^ ]/{p=0} p' | sort -u | sed 's/^/   /'");
        fflush(stdout); puts("  sinks:"); fflush(stdout); sh("wpctl status 2>/dev/null | sed -n '/Sinks:/,/Sources:/p' | head -8 | sed 's/^/   /'");
        fflush(stdout); puts("  recent problems:"); fflush(stdout); sh("journalctl --user --since '-30min' --no-pager 2>/dev/null | grep -i -E 'terminated unexpectedly|No input node|spatial-daemon.*(cannot|error)' | tail -4 | cut -c1-140 | sed 's/^/   /'");
    } else { fprintf(stderr, "usage: spatial on|off|toggle|status|query|strength [N]|lookahead [MS]|doctor\n"); return 1; }
    return 0;
}
