// spatial-daemon: keeps head-tracked spatial audio running for compatible Sony earbuds. Fully event driven.
//
// Event sources (no polling, no timers):
//   PipeWire graph   native registry + node/metadata listeners   default output, earbuds sink, filter node, volume
//   on/off, mode     GFileMonitor on the flag and config folders
//   sensor device    libudev monitor (hidraw)                    earbuds' head tracker appears / disappears
//   Bluetooth        BlueZ D-Bus signals                         device (dis)connected -> ask for the sensor profile
//   tracker          its own stdout                              "no report" twice => sensor stalled => restart it
//   signals          SIGTERM / SIGINT                            clean shutdown
//
// "On"  : while compatible stereo (A2DP) Sony earbuds are the output, run the filter, the head-track helper and the
//         tracker.   "Off": effect fades to plain stereo (helper exits itself), tracker stops, and the filter is
//         removed the moment PipeWire reports nothing is playing through it.
#include <gio/gio.h>
#include <gio/gunixinputstream.h>
#include <glib-unix.h>
#include <libudev.h>
#include <pipewire/pipewire.h>
#include <pipewire/extensions/metadata.h>
#include <spa/param/props.h>
#include <spa/pod/builder.h>
#include <spa/pod/iter.h>
#include <spa/utils/json.h>
#include <signal.h>
#include <cstdarg>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

static std::string HERE, ROOT;                         // HERE = folder of this binary (build/), ROOT = repository root
static std::string RUN, FLAGDIR, FLAG, CONFDIR, STATE_FILE;
static const char* HID_UUID = "00001124-0000-1000-8000-00805f9b34fb";
static const char* FILTER_NAME = "spatial-head-tracked";

static void say(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); putchar('\n'); fflush(stdout);
}
static std::string homeDir() { const char* h = getenv("HOME"); return h ? h : "."; }
static std::string readFile(const std::string& p) { std::ifstream f(p); std::stringstream s; s << f.rdbuf(); return s.str(); }
static bool exists(const std::string& p) { return access(p.c_str(), F_OK) == 0; }

// ------------------------------------------------------------------------------------------ supervised child
struct Daemon;
extern Daemon* G;
struct Child {
    std::string name; GSubprocess* proc = nullptr; std::vector<std::string> argv; std::function<void(const std::string&)> onLine;
    std::function<void()> onExit; FILE* log = nullptr; GCancellable* cancel = nullptr;
    explicit Child(std::string n) : name(std::move(n)) {}
    bool running() const { return proc != nullptr; }
    std::string pidFile() const { return RUN + "/" + name + ".pid"; }

    void start(const std::vector<std::string>& a) {
        if (proc && argv == a) return;
        stop(); argv = a;
        std::vector<const char*> v; for (auto& s : a) v.push_back(s.c_str()); v.push_back(nullptr);
        GError* err = nullptr;
        proc = g_subprocess_newv(v.data(), (GSubprocessFlags)(G_SUBPROCESS_FLAGS_STDOUT_PIPE | G_SUBPROCESS_FLAGS_STDERR_MERGE), &err);
        if (!proc) { say("%s: cannot start: %s", name.c_str(), err->message); g_error_free(err); return; }
        std::ofstream(pidFile()) << g_subprocess_get_identifier(proc);
        log = fopen((RUN + "/" + name + ".log").c_str(), "w");
        cancel = g_cancellable_new();
        auto* in = g_data_input_stream_new(g_subprocess_get_stdout_pipe(proc));
        readLine(in, proc);
        g_subprocess_wait_async(proc, nullptr, (GAsyncReadyCallback)exited, new std::pair<Child*, GSubprocess*>(this, proc));
        say("started %s", name.c_str());
    }
    static void readDone(GObject*, GAsyncResult* res, gpointer d) {
        auto* ctx = (std::tuple<Child*, GSubprocess*, GDataInputStream*>*)d;
        auto [self, p, in] = *ctx; delete ctx;
        gsize len = 0; GError* err = nullptr;
        char* line = g_data_input_stream_read_line_finish(in, res, &len, &err);
        if (err) g_error_free(err);
        if (!line) { g_object_unref(in); return; }                                 // EOF
        if (p == self->proc) { if (self->log) { fprintf(self->log, "%s\n", line); fflush(self->log); } if (self->onLine) self->onLine(line); }
        g_free(line);
        self->readLine(in, p);
    }
    void readLine(GDataInputStream* in, GSubprocess* p) {
        g_data_input_stream_read_line_async(in, G_PRIORITY_DEFAULT, nullptr, readDone, new std::tuple<Child*, GSubprocess*, GDataInputStream*>(this, p, in));
    }
    static void exited(GObject* o, GAsyncResult* res, gpointer d) {
        auto* ctx = (std::pair<Child*, GSubprocess*>*)d; Child* self = ctx->first; GSubprocess* p = ctx->second; delete ctx;
        g_subprocess_wait_finish(p, res, nullptr);
        if (p == self->proc) {
            g_object_unref(self->proc); self->proc = nullptr; self->argv.clear();
            unlink(self->pidFile().c_str()); if (self->log) { fclose(self->log); self->log = nullptr; }
            say("exited %s", self->name.c_str());
            if (self->onExit) self->onExit();
        } else g_object_unref(p);
        (void)o;
    }
    void signal(int sig) { if (proc) g_subprocess_send_signal(proc, sig); }
    void stop() {
        if (!proc) return;
        GSubprocess* p = proc; proc = nullptr; argv.clear(); unlink(pidFile().c_str());
        g_subprocess_send_signal(p, SIGTERM);                         // its exit is reaped by the wait_async callback
        say("stopped %s", name.c_str());
    }
};

// ------------------------------------------------------------------------------------------ PipeWire model
struct NodeInfo {
    uint32_t id = 0; std::string name, cls, desc, group, factory, state;
    int dev = -1; bool smart = false; std::vector<float> vol; bool mute = false;
    pw_node* proxy = nullptr; spa_hook hook{};
};

struct Daemon {
    GMainLoop* loop = nullptr; int exitCode = 0;
    // PipeWire
    pw_loop* pwLoop = nullptr; pw_context* ctx = nullptr; pw_core* core = nullptr; pw_registry* reg = nullptr;
    spa_hook coreHook{}, regHook{};
    std::map<uint32_t, std::unique_ptr<NodeInfo>> nodes; std::set<uint32_t> published;
    pw_metadata* metaDefault = nullptr; pw_metadata* metaFilters = nullptr; spa_hook metaDefaultHook{}, metaFiltersHook{};
    std::string defaultSink;
    // children
    Child pw{"pw"}, ht{"ht"}, tr{"tr"};
    std::map<std::string, bool> capable;                                   // Bluetooth address -> has the head-tracker HID service (cleared on Bluetooth events)
    bool pending = false, hidAttempted = false, hidInFlight = false; int stalls = 0;
    GDBusConnection* bus = nullptr; GFileMonitor *m1 = nullptr, *m2 = nullptr; udev* ud = nullptr; udev_monitor* um = nullptr;
    std::string lastState, lastCompat;

    // ---- plumbing
    void schedule(bool sensor = false) {
        if (sensor) { hidAttempted = false; capable.clear(); }
        if (!pending) { pending = true; g_idle_add([](gpointer d) -> gboolean { auto* s = (Daemon*)d; s->pending = false; s->reconcile(); return G_SOURCE_REMOVE; }, this); }
    }

    // ---- PipeWire events
    static void nodeInfo(void* d, const pw_node_info* info) {
        auto* n = (NodeInfo*)d; Daemon* g = G;
        if (info->props) {                                           // only overwrite what this event actually carries
            auto has = [&](const char* k, std::string& out) { if (const char* v = spa_dict_lookup(info->props, k)) out = v; };
            has("node.name", n->name); has("media.class", n->cls); has("node.description", n->desc);
            has("node.link-group", n->group); has("factory.name", n->factory);
            std::string dv, sm; has("device.id", dv); has("filter.smart", sm);
            if (!dv.empty()) n->dev = atoi(dv.c_str());
            if (!sm.empty()) n->smart = (sm == "true" || sm == "1");
        }
        n->state = pw_node_state_as_string(info->state);
        g->schedule();
    }
    static void nodeParam(void* d, int, uint32_t id, uint32_t, uint32_t, const spa_pod* param) {
        auto* n = (NodeInfo*)d;
        if (id != SPA_PARAM_Props || !param || !spa_pod_is_object(param)) return;
        const spa_pod_object* o = (const spa_pod_object*)param; const spa_pod_prop* p;
        bool changed = false;
        SPA_POD_OBJECT_FOREACH(o, p) {
            if (p->key == SPA_PROP_channelVolumes && spa_pod_is_array(&p->value)) {
                uint32_t cnt = 0; const float* v = (const float*)spa_pod_get_array(&p->value, &cnt);
                std::vector<float> nv(v, v + cnt); if (nv != n->vol) { n->vol = nv; changed = true; }
            } else if (p->key == SPA_PROP_mute && spa_pod_is_bool(&p->value)) {
                bool m; spa_pod_get_bool(&p->value, &m); if (m != n->mute) { n->mute = m; changed = true; }
            }
        }
        if (changed) G->schedule();
    }
    static int metaDefaultProp(void* d, uint32_t, const char* key, const char*, const char* value) {
        (void)d;
        if (key && !strcmp(key, "default.audio.sink")) {
            std::string v = value ? value : ""; std::string name;
            size_t p = v.find("\"name\""); if (p != std::string::npos) { p = v.find('"', v.find(':', p) + 1); size_t e = v.find('"', p + 1); if (p != std::string::npos && e != std::string::npos) name = v.substr(p + 1, e - p - 1); }
            if (name != G->defaultSink) { G->defaultSink = name; G->schedule(); }
        }
        return 0;
    }
    static void regGlobal(void* d, uint32_t id, uint32_t, const char* type, uint32_t, const spa_dict* props) {
        auto* me = (Daemon*)d;
        if (!strcmp(type, PW_TYPE_INTERFACE_Node)) {
            auto n = std::make_unique<NodeInfo>(); n->id = id;
            n->proxy = (pw_node*)pw_registry_bind(me->reg, id, type, PW_VERSION_NODE, 0);
            static const pw_node_events ev = {PW_VERSION_NODE_EVENTS, nodeInfo, nodeParam};
            pw_node_add_listener(n->proxy, &n->hook, &ev, n.get());
            uint32_t ids[1] = {SPA_PARAM_Props}; pw_node_subscribe_params(n->proxy, ids, 1);
            me->nodes[id] = std::move(n);
        } else if (!strcmp(type, PW_TYPE_INTERFACE_Metadata) && props) {
            const char* nm = spa_dict_lookup(props, "metadata.name");
            if (nm && !strcmp(nm, "default")) {
                me->metaDefault = (pw_metadata*)pw_registry_bind(me->reg, id, type, PW_VERSION_METADATA, 0);
                static const pw_metadata_events ev = {PW_VERSION_METADATA_EVENTS, metaDefaultProp};
                pw_metadata_add_listener(me->metaDefault, &me->metaDefaultHook, &ev, me);
            } else if (nm && !strcmp(nm, "filters")) {
                me->metaFilters = (pw_metadata*)pw_registry_bind(me->reg, id, type, PW_VERSION_METADATA, 0);
                me->schedule();
            }
        }
    }
    static void regRemove(void* d, uint32_t id) {
        auto* me = (Daemon*)d; auto it = me->nodes.find(id);
        if (it != me->nodes.end()) { spa_hook_remove(&it->second->hook); pw_proxy_destroy((pw_proxy*)it->second->proxy); me->nodes.erase(it); me->published.erase(id); me->schedule(); }
    }
    static void coreError(void* d, uint32_t id, int, int res, const char* msg) {
        auto* me = (Daemon*)d;
        if (id == PW_ID_CORE && res == -EPIPE) { say("PipeWire went away (%s); exiting for a clean restart", msg); me->exitCode = 1; me->shutdown(); }
    }
    bool connectPipeWire() {
        pw_init(nullptr, nullptr);
        pwLoop = pw_loop_new(nullptr); ctx = pw_context_new(pwLoop, nullptr, 0);
        core = ctx ? pw_context_connect(ctx, nullptr, 0) : nullptr;
        if (!core) return false;
        static const pw_core_events cev = {PW_VERSION_CORE_EVENTS, nullptr, nullptr, nullptr, coreError};
        pw_core_add_listener(core, &coreHook, &cev, this);
        reg = pw_core_get_registry(core, PW_VERSION_REGISTRY, 0);
        static const pw_registry_events rev = {PW_VERSION_REGISTRY_EVENTS, regGlobal, regRemove};
        pw_registry_add_listener(reg, &regHook, &rev, this);
        pw_loop_enter(pwLoop);
        g_unix_fd_add(pw_loop_get_fd(pwLoop), G_IO_IN, [](gint, GIOCondition, gpointer d) -> gboolean { pw_loop_iterate(((Daemon*)d)->pwLoop, 0); return G_SOURCE_CONTINUE; }, this);
        return true;
    }
    NodeInfo* byName(const std::string& n) { for (auto& kv : nodes) if (kv.second->name == n) return kv.second.get(); return nullptr; }

    // ---- decisions
    // Selected output = stereo (A2DP) Sony earbuds that support head tracking.
    // TESTING OVERRIDE: `spatial force on` (file CONFDIR/force) skips the hardware checks and attaches the filter to whatever
    // output is selected, so the effect can be tried without the earbuds (use test packets for the head pose).
    static bool forced() { return exists(CONFDIR + "/force"); }
    // Any Sony Bluetooth device that offers the HID service carrying the head-tracker sensor (what SonyTrackerLinux reads) is supported:
    // WF/WH-1000XM5/XM6, LinkBuds, ULT WEAR and so on. No model list to maintain.
    bool trackerCapable(const std::string& nodeName) {
        std::string mac = nodeName.substr(strlen("bluez_output.")); size_t dot = mac.find('.', 17); if (dot != std::string::npos) mac.resize(dot);
        for (auto& c : mac) c = c == '_' ? ':' : toupper(c);
        auto it = capable.find(mac); if (it != capable.end()) return it->second;
        bool ok = false;
        if (bus) {
            GVariant* r = g_dbus_connection_call_sync(bus, "org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr, G_VARIANT_TYPE("(a{oa{sa{sv}}})"), G_DBUS_CALL_FLAGS_NONE, 3000, nullptr, nullptr);
            if (r) {
                GVariantIter* oi; const char* path; GVariant* ifaces; g_variant_get(r, "(a{oa{sa{sv}}})", &oi);
                while (g_variant_iter_next(oi, "{&o@a{sa{sv}}}", &path, &ifaces)) {
                    if (GVariant* dev = g_variant_lookup_value(ifaces, "org.bluez.Device1", G_VARIANT_TYPE("a{sv}"))) {
                        const char* addr = ""; const char* modalias = ""; g_variant_lookup(dev, "Address", "&s", &addr); g_variant_lookup(dev, "Modalias", "&s", &modalias);
                        std::string a = addr, ma = modalias; for (auto& c : a) c = toupper(c); for (auto& c : ma) c = tolower(c);
                        if (a == mac && ma.rfind("usb:v054c", 0) == 0)
                            if (GVariant* uuids = g_variant_lookup_value(dev, "UUIDs", G_VARIANT_TYPE("as"))) {
                                gsize n; const char** u = g_variant_get_strv(uuids, &n); for (gsize i = 0; i < n; ++i) if (!strcmp(u[i], HID_UUID)) ok = true; g_free(u); g_variant_unref(uuids);
                            }
                        g_variant_unref(dev);
                    }
                    g_variant_unref(ifaces);
                }
                g_variant_iter_free(oi); g_variant_unref(r);
            }
        }
        return capable[mac] = ok;
    }
    bool compatible() {
        if (forced()) { if (lastCompat != "forced") { lastCompat = "forced"; say("compatible: forced (hardware checks disabled)"); } return !defaultSink.empty(); }
        bool ok = false; std::string why;
        if (defaultSink.rfind("bluez_output.", 0) != 0) why = "default output is '" + defaultSink + "'";
        else if (NodeInfo* n = byName(defaultSink)) {
            bool a2dp = false; for (auto& kv : nodes) if (kv.second->dev == n->dev && kv.second->factory == "api.bluez5.a2dp.sink") a2dp = true;
            if (!a2dp) why = "no A2DP sink for device " + std::to_string(n->dev);
            else if (!trackerCapable(n->name)) why = "'" + n->desc + "' has no Sony head-tracker sensor service";
            else ok = true;
        } else why = "default output node '" + defaultSink + "' not seen yet";
        std::string key = ok ? "yes" : "no: " + why;
        if (key != lastCompat) { lastCompat = key; say("compatible: %s", key.c_str()); }
        return ok;
    }
    static std::string sensorNode() {
        DIR* d = opendir("/sys/class/hidraw"); if (!d) return "";
        std::string found; while (auto* e = readdir(d)) {
            if (strncmp(e->d_name, "hidraw", 6)) continue;
            std::string u = readFile(std::string("/sys/class/hidraw/") + e->d_name + "/device/uevent");
            for (auto& c : u) c = toupper(c);
            if (u.find("HID_ID=0005:0000054C") != std::string::npos) { std::string p = std::string("/dev/") + e->d_name; if (access(p.c_str(), R_OK) == 0) found = p; }
        }
        closedir(d); return found;
    }
    void askForSensorProfile() {                                         // once per device event; success arrives as a udev event
        if (hidAttempted || hidInFlight || !bus) return;
        hidAttempted = true;
        GError* err = nullptr;
        GVariant* r = g_dbus_connection_call_sync(bus, "org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr, G_VARIANT_TYPE("(a{oa{sa{sv}}})"), G_DBUS_CALL_FLAGS_NONE, 3000, nullptr, &err);
        if (!r) { if (err) g_error_free(err); return; }
        GVariantIter* oi; const char* path; GVariant* ifaces; g_variant_get(r, "(a{oa{sa{sv}}})", &oi);
        while (g_variant_iter_next(oi, "{&o@a{sa{sv}}}", &path, &ifaces)) {
            GVariant* dev = g_variant_lookup_value(ifaces, "org.bluez.Device1", G_VARIANT_TYPE("a{sv}"));
            if (dev) {
                gboolean conn = FALSE; const char* modalias = ""; g_variant_lookup(dev, "Connected", "b", &conn); g_variant_lookup(dev, "Modalias", "&s", &modalias);
                GVariant* uuids = g_variant_lookup_value(dev, "UUIDs", G_VARIANT_TYPE("as")); bool hasHid = false;
                if (uuids) { gsize n; const char** u = g_variant_get_strv(uuids, &n); for (gsize i = 0; i < n; ++i) if (!strcmp(u[i], HID_UUID)) hasHid = true; g_free(u); g_variant_unref(uuids); }
                std::string ma = modalias; for (auto& c : ma) c = tolower(c);
                if (conn && hasHid && ma.rfind("usb:v054c", 0) == 0)
                    g_dbus_connection_call(bus, "org.bluez", path, "org.bluez.Device1", "ConnectProfile", g_variant_new("(s)", HID_UUID), nullptr, G_DBUS_CALL_FLAGS_NONE, 15000, nullptr,
                        [](GObject* o, GAsyncResult* res, gpointer d) {      // refused (sensor still going away): arm another try for the next BlueZ event
                            auto* me = (Daemon*)d; GError* e = nullptr; GVariant* r = g_dbus_connection_call_finish(G_DBUS_CONNECTION(o), res, &e);
                            if (r) g_variant_unref(r);
                            me->hidInFlight = false;
                            if (e) { me->hidAttempted = false; g_error_free(e); } }, this);
                hidInFlight = true;
                g_variant_unref(dev);
            }
            g_variant_unref(ifaces);
        }
        g_variant_iter_free(oi); g_variant_unref(r);
    }
    void setProps(NodeInfo* n, const std::function<void(spa_pod_builder*, spa_pod_frame*)>& fill) {
        uint8_t buf[1024]; spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof buf); spa_pod_frame f;
        spa_pod_builder_push_object(&b, &f, SPA_TYPE_OBJECT_Props, SPA_PARAM_Props); fill(&b, &f);
        pw_node_set_param(n->proxy, SPA_PARAM_Props, 0, (spa_pod*)spa_pod_builder_pop(&b, &f));
    }
    void pinVolume(NodeInfo* n) {                                        // the filter itself must never attenuate
        bool bad = n->mute; for (float v : n->vol) if (v < 0.99f || v > 1.01f) bad = true;
        if (!bad || n->vol.empty()) return;
        std::vector<float> ones(n->vol.size(), 1.0f);
        setProps(n, [&](spa_pod_builder* b, spa_pod_frame*) {
            spa_pod_builder_prop(b, SPA_PROP_channelVolumes, 0); spa_pod_builder_array(b, sizeof(float), SPA_TYPE_Float, (uint32_t)ones.size(), ones.data());
            spa_pod_builder_prop(b, SPA_PROP_mute, 0); spa_pod_builder_bool(b, false);
        });
    }
    void orderFilters() {                                                // our filter goes in front of Bluetooth's own loopback
        if (!metaFilters) return;
        for (auto& kv : nodes) {
            NodeInfo* n = kv.second.get();
            if (n->name.rfind("bluez_output.", 0) == 0 && n->smart && !published.count(n->id)) {
                published.insert(n->id);
                pw_metadata_set_property(metaFilters, n->id, "filter.smart.after", "Spa:String:JSON", (std::string("[\"") + FILTER_NAME + "\"]").c_str());
            }
        }
    }
    void writeState(bool compat, bool enabled, bool force_off = false) {
        char buf[128]; snprintf(buf, sizeof buf, "{\"enabled\": %s, \"compatible\": %s, \"active\": %s}", enabled ? "true" : "false", compat ? "true" : "false", (pw.running() && compat && !force_off) ? "true" : "false");
        if (lastState == buf) return;
        lastState = buf;
        g_file_set_contents(STATE_FILE.c_str(), buf, -1, nullptr);       // atomic rename: watchers see one change
    }
    void reconcile() {
        bool compat = compatible(), enabled = exists(FLAG);
        if (!compat) { ht.stop(); tr.stop(); pw.stop(); }
        else {
            std::string conf = HERE + "/head-tracked-ambi.conf";
            if (forced()) {                                              // retarget the smart filter at the selected output
                std::string text = readFile(conf); const std::string key = "bluez5.sink-loopback-target = true";
                size_t p = text.find(key); if (p != std::string::npos) text.replace(p, key.size(), "node.name = \"" + defaultSink + "\"");
                conf = RUN + "/force-" + std::to_string(std::hash<std::string>{}(defaultSink) % 100000) + ".conf"; g_file_set_contents(conf.c_str(), text.c_str(), -1, nullptr);
            }
            if (enabled) pw.start({"pipewire", "-c", conf});
            NodeInfo* f = byName("spatial_test_in");
            // Off: while audio is playing the filter stays in the path as a plain-stereo passthrough (moving a playing stream can interrupt it);
            // as soon as nothing is playing it is removed, and streams are re-linked to the earbuds while silent.
            if (!enabled && pw.running() && (!f || f->state != "running")) { pw.stop(); f = nullptr; }
            if (f) { pinVolume(f); orderFilters(); }
            if (enabled && f) {
                ht.start({HERE + "/head-track", "--node", std::to_string(f->id)});
                std::string sensor = sensorNode();
                if (!sensor.empty()) tr.start({ROOT + "/third_party/SonyTrackerLinux/sony-tracker", "-d", sensor}); else { tr.stop(); askForSensorProfile(); }
            } else if (!enabled) {
                if (ht.running()) ht.signal(SIGUSR2);                    // helper glides to plain stereo and exits -> we run again
                else tr.stop();                                          // the filter goes once playback stops (above)
            }
        }
        writeState(compat, enabled);
    }
    void trackerLine(const std::string& l) {
        if (l.find("no report") != std::string::npos) {
            if (++stalls >= 2) { say("sensor stalled; restarting tracker"); stalls = 0; ht.signal(SIGHUP); tr.stop(); schedule(); }
        } else stalls = 0;
    }
    void shutdown() {
        ht.stop(); tr.stop(); pw.stop(); writeState(false, false, true);
        g_main_loop_quit(loop);
    }

    // ---- start
    int run() {
        loop = g_main_loop_new(nullptr, FALSE);
        tr.onLine = [this](const std::string& l) { trackerLine(l); };
        tr.onExit = [this] { schedule(); }; ht.onExit = [this] { schedule(); }; pw.onExit = [this] { schedule(); };
        if (!connectPipeWire()) { say("cannot connect to PipeWire"); return 1; }
        m1 = g_file_monitor_directory(g_file_new_for_path(FLAGDIR.c_str()), G_FILE_MONITOR_NONE, nullptr, nullptr);
        m2 = g_file_monitor_directory(g_file_new_for_path(CONFDIR.c_str()), G_FILE_MONITOR_NONE, nullptr, nullptr);
        for (GFileMonitor* m : {m1, m2}) if (m) g_signal_connect(m, "changed", G_CALLBACK(+[](GFileMonitor*, GFile*, GFile*, GFileMonitorEvent, gpointer d) { ((Daemon*)d)->schedule(); }), this);
        ud = udev_new(); um = udev_monitor_new_from_netlink(ud, "udev");
        udev_monitor_filter_add_match_subsystem_devtype(um, "hidraw", nullptr); udev_monitor_enable_receiving(um);
        g_unix_fd_add(udev_monitor_get_fd(um), G_IO_IN, [](gint, GIOCondition, gpointer d) -> gboolean {
            auto* me = (Daemon*)d; if (auto* dev = udev_monitor_receive_device(me->um)) udev_device_unref(dev); me->schedule(true); return G_SOURCE_CONTINUE; }, this);
        bus = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, nullptr);
        if (bus) g_dbus_connection_signal_subscribe(bus, "org.bluez", "org.freedesktop.DBus.Properties", "PropertiesChanged", nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
            [](GDBusConnection*, const gchar*, const gchar*, const gchar*, const gchar*, GVariant* p, gpointer d) {
                const char* iface; GVariant *changed, *invalid; g_variant_get(p, "(&s@a{sv}@as)", &iface, &changed, &invalid);
                if (!strcmp(iface, "org.bluez.Device1")) { auto* me = (Daemon*)d; if (GVariant* c = g_variant_lookup_value(changed, "Connected", nullptr)) { g_variant_unref(c); me->schedule(true); } else if (me->sensorNode().empty()) me->schedule(true); }
                g_variant_unref(changed); g_variant_unref(invalid); }, this, nullptr);
        g_unix_signal_add(SIGTERM, [](gpointer d) -> gboolean { ((Daemon*)d)->shutdown(); return G_SOURCE_REMOVE; }, this);
        g_unix_signal_add(SIGINT, [](gpointer d) -> gboolean { ((Daemon*)d)->shutdown(); return G_SOURCE_REMOVE; }, this);
        schedule();
        g_main_loop_run(loop);
        return exitCode;
    }
};
Daemon* G = nullptr;

int main(int, char** argv) {
    char buf[4096]; ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1); buf[n > 0 ? n : 0] = 0;
    HERE = std::string(buf); HERE = HERE.substr(0, HERE.rfind('/')); ROOT = HERE + "/..";
    const char* xr = getenv("XDG_RUNTIME_DIR"); RUN = std::string(xr ? xr : "/tmp") + "/spatial-audio"; g_mkdir_with_parents(RUN.c_str(), 0700);
    FLAGDIR = homeDir() + "/.local/state/spatial-audio"; g_mkdir_with_parents(FLAGDIR.c_str(), 0700); FLAG = FLAGDIR + "/enabled";
    CONFDIR = homeDir() + "/.config/spatial-audio"; g_mkdir_with_parents(CONFDIR.c_str(), 0700);
    STATE_FILE = RUN + "/state.json";
    Daemon d; G = &d; (void)argv;
    return d.run();
}
