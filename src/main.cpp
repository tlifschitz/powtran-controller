// powtran-ctl — CLI and cycle runner for a Powtran PI150 VFD.
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>

extern "C" {
#include <unistd.h>
}

#include "vfd.hpp"

static volatile sig_atomic_t gAbort = 0;
static void onSignal(int) { gAbort = 1; }

static double now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// a lost monitor sample must not abort the test cycle (EMI is transient);
// only give up after several consecutive failures
static bool waitMonitor(Vfd &vfd, double sec, double t0) {
    double end = now() + sec;
    int fails  = 0;
    while (!gAbort) {
        if (vfd.monitorLine(now() - t0)) fails = 0;
        else if (++fails >= 3) return false;
        double left = end - now();
        if (left <= 0) break;
        usleep((useconds_t)((left < 1.0 ? left : 1.0) * 1e6));
    }
    return true;
}

static int runCycle(Vfd &vfd, const Config &cfg, const std::vector<Step> &steps) {
    double t0 = now();
    for (int it = 0; (cfg.loop == 0 || it < cfg.loop) && !gAbort; ++it) {
        std::string total = cfg.loop ? "/" + std::to_string(cfg.loop) : " (forever)";
        printf("--- cycle %d%s ---\n", it + 1, total.c_str());
        for (const Step &s : steps) {
            if (gAbort) break;
            bool ok = true;
            switch (s.cmd) {  // no default: -Wswitch flags any unhandled Cmd
            case Cmd::Run: ok = vfd.runRpm(s.a, s.b); break;
            case Cmd::Hz: ok = vfd.runHz(std::fabs(s.a), s.b, s.a < 0); break;
            case Cmd::Stop: ok = vfd.stop(s.b); break;
            case Cmd::Coast: ok = vfd.coast(); break;
            case Cmd::Reset: ok = vfd.reset(); break;
            case Cmd::Wait: ok = waitMonitor(vfd, s.a, t0); break;
            case Cmd::Write:
                printf("write 0x%04X = %u\n", (unsigned)s.a, (unsigned)s.b);
                ok = vfd.write((uint16_t)s.a, (uint16_t)s.b);
                break;
            }
            if (!ok) { vfd.stop(NAN); return 1; }
        }
    }
    if (gAbort) {
        printf("\ninterrupted -> decel stop\n");
        vfd.stop(NAN);
        return 130;
    }
    return 0;
}

static int usage() {
    fprintf(stderr,
        "usage: powtran-ctl [-p port] [-b baud] [-a slave] [--rts down|up|none]\n"
        "                   [--rpm-per-hz N] <cmd>\n"
        "  status                    show run state, freq, rpm, fault\n"
        "  cycle <file.yaml>         execute a cycle file\n"
        "  run <rpm> [ramp_s]        start motor (negative rpm = reverse)\n"
        "  hz <hz> [ramp_s]          start motor by frequency\n"
        "  stop [ramp_s]             decelerate and stop\n"
        "  coast                     free-wheel stop\n"
        "  reset                     fault reset\n"
        "  read <addr> [count]       read holding registers (0x.. ok)\n"
        "  write <addr> <value>      write one register\n"
        "  setup [off]               set F0.11/F0.03 for comm control (EEPROM)\n"
        "ramp_s = accel/decel time 0 -> max freq (VFD native semantics)\n");
    return 2;
}

int main(int argc, char **argv) {
    Config cfg;
    int i = 1;
    for (; i < argc && argv[i][0] == '-'; ++i) {
        std::string o = argv[i];
        if (o == "-p" && i + 1 < argc) cfg.port = argv[++i];
        else if (o == "-b" && i + 1 < argc) cfg.baud = atoi(argv[++i]);
        else if (o == "-a" && i + 1 < argc) cfg.slave = atoi(argv[++i]);
        else if (o == "--rts" && i + 1 < argc) cfg.rts = argv[++i];
        else if (o == "--rpm-per-hz" && i + 1 < argc) cfg.rpmPerHz = atof(argv[++i]);
        else return usage();
    }
    if (i >= argc) return usage();
    std::string cmd = argv[i++];
    auto arg = [&](int k) { return i + k < argc ? argv[i + k] : nullptr; };
    auto argNum = [&](int k, double dflt) { return arg(k) ? atof(arg(k)) : dflt; };

    std::vector<Step> steps;
    if (cmd == "cycle") {
        if (!arg(0)) return usage();
        std::ifstream f(arg(0));
        if (!f) { fprintf(stderr, "cannot open %s\n", arg(0)); return 1; }
        std::string err;
        if (!parseCycleText(f, cfg, steps, err)) {
            fprintf(stderr, "%s: %s\n", arg(0), err.c_str());
            return 1;
        }
        if (steps.empty()) { fprintf(stderr, "%s: no cycle steps\n", arg(0)); return 1; }
    }

    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    Vfd vfd;
    if (!vfd.open(cfg)) return 1;

    if (cmd == "cycle") return runCycle(vfd, cfg, steps);
    if (cmd == "status") return vfd.status() ? 0 : 1;
    if (cmd == "run") return arg(0) ? (vfd.runRpm(atof(arg(0)), argNum(1, NAN)) ? 0 : 1) : usage();
    if (cmd == "hz") {
        if (!arg(0)) return usage();
        double hz = atof(arg(0));
        return vfd.runHz(std::fabs(hz), argNum(1, NAN), hz < 0) ? 0 : 1;
    }
    if (cmd == "stop") return vfd.stop(argNum(0, NAN)) ? 0 : 1;
    if (cmd == "coast") return vfd.coast() ? 0 : 1;
    if (cmd == "reset") return vfd.reset() ? 0 : 1;
    if (cmd == "read") {
        if (!arg(0)) return usage();
        uint16_t addr = (uint16_t)strtoul(arg(0), nullptr, 0);
        int n         = arg(1) ? atoi(arg(1)) : 1;
        std::vector<uint16_t> v(n);
        if (!vfd.read(addr, v.data(), n)) return 1;
        for (int k = 0; k < n; ++k)
            printf("[0x%04X] = 0x%04X (%u)\n", addr + k, v[k], v[k]);
        return 0;
    }
    if (cmd == "write") {
        if (!arg(0) || !arg(1)) return usage();
        uint16_t addr = (uint16_t)strtoul(arg(0), nullptr, 0);
        uint16_t val  = (uint16_t)strtoul(arg(1), nullptr, 0);
        return vfd.write(addr, val) ? 0 : 1;
    }
    if (cmd == "setup") {
        bool off = arg(0) && std::string(arg(0)) == "off";
        // EEPROM writes, one-time: F0.11 command source, F0.03 frequency source
        struct { uint16_t addr, want; const char *name; } p[] = {
            {EE_F0_11, (uint16_t)(off ? 0 : 2), "F0.11 command source"},
            {EE_F0_03, (uint16_t)(off ? 1 : 9), "F0.03 frequency source"},
        };
        for (auto &e : p) {
            uint16_t cur;
            if (!vfd.read(e.addr, &cur)) return 1;
            if (cur == e.want) { printf("%s already %u\n", e.name, cur); continue; }
            if (!vfd.write(e.addr, e.want)) return 1;
            printf("%s: %u -> %u\n", e.name, cur, e.want);
        }
        printf("VFD %s under communication control\n", off ? "no longer" : "now");
        return 0;
    }
    return usage();
}
