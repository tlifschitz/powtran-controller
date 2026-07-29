// powtran-ctl — Powtran PI150 VFD control over Modbus RTU (RS485).
//
// Register map (PI150/PI500/PI550 shared protocol, verified on device):
//   0x2000 W  control: 1 fwd run, 2 rev run, 3 fwd jog, 4 rev jog,
//                      5 coast stop, 6 decel stop, 7 fault reset
//   0x1000 W  frequency setting, 0..10000 = 0..100% of max freq (F0.19)
//   0x1000 R  monitor block: freq, set freq, bus V, out V, out I, power,
//             torque, speed(rpm), DI, DO  (0.01Hz/0.01Hz/0.1V/1V/0.01A units)
//   0x3000 R  status: 1 fwd run, 2 rev run, 3 stop
//   0x8000 R  fault code (0 = none)
//   Fg.ii  RW parameter at 0xF0'gii' (EEPROM) or 0x0'gii' (RAM only)

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Config + cycle parsing (yaml subset), testable without hardware
// ---------------------------------------------------------------------------

struct Config {
    std::string port = "/dev/ttyS1";
    int baud         = 9600;  // VFD F9.00=5, F9.01=0 -> 9600 8N2
    int slave        = 1;
    std::string rts  = "down";  // rs485 direction via RTS: down|up|none
    double rpmPerHz  = 30.0;    // Kaifa moto MS802-4, 4-pole: 1500 rpm @ 50 Hz
    double maxRpm    = 0;       // safety clamp, 0 = disabled
    int loop         = 1;       // cycle repetitions, 0 = forever
};

struct Step {
    std::string cmd;  // run | hz | stop | coast | wait | reset | write
    double a = 0, b = NAN;  // b = NAN means "not given"
};

static std::string trim(const std::string &s) {
    size_t b = s.find_first_not_of(" \t\r");
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(" \t\r") - b + 1);
}

// ponytail: hand-rolled yaml subset (key: value + one "cycle:" list),
// switch to yaml-cpp if the format ever needs nesting
static bool parseCycleText(std::istream &in, Config &cfg, std::vector<Step> &steps, std::string &err) {
    std::string line;
    bool inCycle = false;
    int ln       = 0;

    while (std::getline(in, line)) {
        ++ln;
        size_t h = line.find('#');
        if (h != std::string::npos) line.erase(h);
        std::string t = trim(line);
        if (t.empty()) continue;

        if (t == "cycle:") {
            inCycle = true;
            continue;
        }

        if (inCycle && t[0] == '-') {
            std::istringstream ss(trim(t.substr(1)));
            Step s;
            ss >> s.cmd;
            std::string a1, a2;
            ss >> a1 >> a2;

            auto num = [&](const std::string &v, double &out) {
                char *end;
                out = std::strtod(v.c_str(), &end);
                return *end == '\0' && !v.empty();
            };
            auto reg = [&](const std::string &v, double &out) {  // accepts 0x...
                char *end;
                out = (double)std::strtoul(v.c_str(), &end, 0);
                return *end == '\0' && !v.empty();
            };

            bool ok = false;
            if (s.cmd == "run" || s.cmd == "hz")
                ok = num(a1, s.a) && (a2.empty() || num(a2, s.b));
            else if (s.cmd == "wait")
                ok = num(a1, s.a) && s.a > 0 && a2.empty();
            else if (s.cmd == "stop")
                ok = a1.empty() ? true : (num(a1, s.b) && a2.empty());
            else if (s.cmd == "coast" || s.cmd == "reset")
                ok = a1.empty();
            else if (s.cmd == "write")
                ok = reg(a1, s.a) && reg(a2, s.b);

            if (!ok) {
                err = "line " + std::to_string(ln) + ": bad step '" + t + "'";
                return false;
            }
            steps.push_back(s);
            continue;
        }

        size_t c = t.find(':');
        if (c == std::string::npos) {
            err = "line " + std::to_string(ln) + ": expected 'key: value'";
            return false;
        }
        std::string key = trim(t.substr(0, c)), val = trim(t.substr(c + 1));
        inCycle = false;
        if (key == "port") cfg.port = val;
        else if (key == "baud") cfg.baud = std::atoi(val.c_str());
        else if (key == "slave") cfg.slave = std::atoi(val.c_str());
        else if (key == "rts") cfg.rts = val;
        else if (key == "rpm_per_hz") cfg.rpmPerHz = std::atof(val.c_str());
        else if (key == "max_rpm") cfg.maxRpm = std::atof(val.c_str());
        else if (key == "loop") cfg.loop = std::atoi(val.c_str());
        else {
            err = "line " + std::to_string(ln) + ": unknown key '" + key + "'";
            return false;
        }
    }
    return true;
}

// rpm -> 0x1000 permille-of-max value (always positive; direction = run cmd)
static int rpmToPct(double rpm, double rpmPerHz, double maxFreqHz, bool &clamped) {
    double hz  = std::fabs(rpm) / rpmPerHz;
    int pct    = (int)std::lround(hz / maxFreqHz * 10000.0);
    clamped    = pct > 10000;
    return clamped ? 10000 : pct;
}

#ifdef PARSER_TEST

int main() {
    std::istringstream in(
        "# demo\n"
        "rpm_per_hz: 60\n"
        "loop: 3\n"
        "cycle:\n"
        "  - run 3000 1\n"
        "  - wait 2\n"
        "  - run -1500 0.5   # reverse\n"
        "  - hz 25\n"
        "  - write 0x000D 10\n"
        "  - stop 1\n"
        "  - coast\n");
    Config cfg;
    std::vector<Step> steps;
    std::string err;
    assert(parseCycleText(in, cfg, steps, err) && err.empty());
    assert(cfg.loop == 3 && cfg.rpmPerHz == 60.0);
    assert(steps.size() == 7);
    assert(steps[0].cmd == "run" && steps[0].a == 3000 && steps[0].b == 1);
    assert(steps[1].cmd == "wait" && steps[1].a == 2);
    assert(steps[2].a == -1500 && steps[2].b == 0.5);
    assert(steps[3].cmd == "hz" && steps[3].a == 25 && std::isnan(steps[3].b));
    assert(steps[4].cmd == "write" && steps[4].a == 0x000D && steps[4].b == 10);
    assert(steps[5].cmd == "stop" && steps[5].b == 1);
    assert(steps[6].cmd == "coast");

    std::istringstream bad("cycle:\n  - fly 100\n");
    Config c2;
    std::vector<Step> s2;
    assert(!parseCycleText(bad, c2, s2, err) && !err.empty());

    bool cl;
    assert(rpmToPct(3000, 60, 50, cl) == 10000 && !cl);
    assert(rpmToPct(-1500, 60, 50, cl) == 5000 && !cl);
    assert(rpmToPct(6000, 60, 50, cl) == 10000 && cl);

    puts("parser tests OK");
    return 0;
}

#else  // real tool

#include <csignal>
#include <ctime>
#include <unistd.h>

extern "C" {
#include <modbus/modbus.h>
}

// registers
enum : uint16_t {
    REG_FREQ_SET = 0x1000,  // write
    REG_MONITOR  = 0x1000,  // read: freq, setfreq, busV, outV, I, P, torque, rpm
    REG_CONTROL  = 0x2000,
    REG_STATUS   = 0x3000,
    REG_FAULT    = 0x8000,
    RAM_ACCEL    = 0x000D,  // F0.13, RAM copy (no EEPROM wear)
    RAM_DECEL    = 0x000E,  // F0.14
    EE_F0_03     = 0xF003,  // frequency source (9 = communication)
    EE_F0_11     = 0xF00B,  // command source (2 = communication)
    EE_F0_15     = 0xF00F,  // ramp time unit: 0=1s 1=0.1s 2=0.01s
    EE_F0_19     = 0xF013,  // max frequency, 0.01 Hz
};
enum : uint16_t { CMD_FWD = 1, CMD_REV = 2, CMD_COAST = 5, CMD_STOP = 6, CMD_RESET = 7 };

static volatile sig_atomic_t gAbort = 0;
static void onSignal(int) { gAbort = 1; }

class Vfd {
public:
    bool open(const Config &cfg) {
        mCfg = cfg;
        mCtx = modbus_new_rtu(cfg.port.c_str(), cfg.baud, 'N', 8, 2);
        if (!mCtx) { fprintf(stderr, "modbus_new_rtu failed\n"); return false; }
        modbus_set_slave(mCtx, cfg.slave);
        if (cfg.rts != "none")
            modbus_rtu_set_rts(mCtx, cfg.rts == "up" ? MODBUS_RTU_RTS_UP : MODBUS_RTU_RTS_DOWN);
        modbus_set_response_timeout(mCtx, 1, 0);
        if (modbus_connect(mCtx) == -1) {
            fprintf(stderr, "connect %s: %s\n", cfg.port.c_str(), modbus_strerror(errno));
            return false;
        }
        return true;
    }
    ~Vfd() {
        if (mCtx) { modbus_close(mCtx); modbus_free(mCtx); }
    }

    // VFD switching noise (EMI) puts junk bytes on the bus while the motor
    // runs: flush before every attempt and retry a few times. Reads get more
    // attempts than writes because telemetry polls hit the noise most often.
    bool read(uint16_t addr, uint16_t *dst, int n = 1) {
        for (int t = 0; t < kReadAttempts; ++t) {
            modbus_flush(mCtx);
            if (modbus_read_registers(mCtx, addr, n, dst) == n) { gap(); return true; }
            gap();
        }
        fprintf(stderr, "read 0x%04X: %s\n", addr, modbus_strerror(errno));
        return false;
    }
    bool write(uint16_t addr, uint16_t val) {
        for (int t = 0; t < kWriteAttempts; ++t) {
            modbus_flush(mCtx);
            if (modbus_write_register(mCtx, addr, val) == 1) { gap(); return true; }
            gap();
        }
        fprintf(stderr, "write 0x%04X=%u: %s\n", addr, val, modbus_strerror(errno));
        return false;
    }

    // reads scaling params from the VFD (once)
    bool loadScaling() {
        if (mMaxFreqHz > 0) return true;
        uint16_t v;
        if (!read(EE_F0_19, &v)) return false;
        mMaxFreqHz = v / 100.0;
        if (!read(EE_F0_15, &v)) return false;
        mRampUnit = (v == 0) ? 1.0 : (v == 1) ? 0.1 : 0.01;
        printf("vfd: max freq %.2f Hz, ramp unit %.2g s\n", mMaxFreqHz, mRampUnit);
        return true;
    }

    bool setRamp(double sec, bool accel, bool decel) {
        long ticks = std::lround(sec / mRampUnit);
        if (ticks > kMaxRampTicks) ticks = kMaxRampTicks;
        if (accel && !write(RAM_ACCEL, (uint16_t)ticks)) return false;
        if (decel && !write(RAM_DECEL, (uint16_t)ticks)) return false;
        return true;
    }

    bool runRpm(double rpm, double ramp) {
        if (!loadScaling()) return false;
        if (mCfg.maxRpm > 0 && std::fabs(rpm) > mCfg.maxRpm) {
            printf("clamp: %g -> %g rpm (max_rpm)\n", rpm, mCfg.maxRpm);
            rpm = rpm < 0 ? -mCfg.maxRpm : mCfg.maxRpm;
        }
        return runHz(std::fabs(rpm) / mCfg.rpmPerHz, ramp, rpm < 0);
    }

    bool runHz(double hz, double ramp, bool reverse) {
        if (!loadScaling()) return false;
        bool clamped;
        int pct = rpmToPct(hz * mCfg.rpmPerHz, mCfg.rpmPerHz, mMaxFreqHz, clamped);
        if (clamped) printf("clamp: %.2f Hz -> %.2f Hz (max freq)\n", hz, mMaxFreqHz);
        printf("run: %.2f Hz (%s, %.1f%%)%s\n", pct * mMaxFreqHz / 10000.0,
               reverse ? "rev" : "fwd", pct / 100.0,
               std::isnan(ramp) ? "" : ", ramp set");
        if (!std::isnan(ramp) && !setRamp(ramp, true, true)) return false;
        if (!write(REG_FREQ_SET, (uint16_t)pct)) return false;
        return write(REG_CONTROL, reverse ? CMD_REV : CMD_FWD);
    }

    bool stop(double ramp) {
        if (!std::isnan(ramp)) {
            if (!loadScaling() || !setRamp(ramp, false, true)) return false;
        }
        printf("stop (decel)\n");
        return write(REG_CONTROL, CMD_STOP);
    }
    bool coast() { printf("stop (coast)\n"); return write(REG_CONTROL, CMD_COAST); }
    bool reset() { printf("fault reset\n"); return write(REG_CONTROL, CMD_RESET); }

    // monitor block, verified live: [0] setpoint echo 0.01%, [1] running
    // freq 0.01Hz, [2] bus 0.1V, [3] out V, [4] out 0.01A, [5] power 0.1kW,
    // [6] torque 0.1%, [7] rotor speed estimate 0.01Hz (slip-adjusted)
    bool monitorLine(double t) {
        uint16_t m[8], st;
        if (!read(REG_MONITOR, m, 8) || !read(REG_STATUS, &st)) return false;
        const char *s = st == 1 ? "fwd" : st == 2 ? "rev" : st == 3 ? "stop" : "?";
        printf("t=%6.1f  st=%-4s f=%6.2fHz set=%5.1f%% bus=%5.1fV out=%3uV I=%5.2fA "
               "P=%.1fkW T=%4.1f%% rpm=%.0f\n",
               t, s, m[1] / 100.0, m[0] / 100.0, m[2] / 10.0, m[3], m[4] / 100.0,
               m[5] / 10.0, m[6] / 10.0, m[7] / 100.0 * mCfg.rpmPerHz);
        fflush(stdout);
        return true;
    }

    bool status() {
        uint16_t m[10], st, flt;
        if (!read(REG_MONITOR, m, 10) || !read(REG_STATUS, &st) || !read(REG_FAULT, &flt))
            return false;
        const char *s = st == 1 ? "running fwd" : st == 2 ? "running rev" : st == 3 ? "stopped" : "unknown";
        printf("status:      %s (%u)\n", s, st);
        printf("fault:       0x%04X%s\n", flt, flt ? "  <-- FAULT" : " (none)");
        printf("freq:        %.2f Hz (setpoint %.1f %%)\n", m[1] / 100.0, m[0] / 100.0);
        printf("speed:       %.0f rpm (slip-adjusted estimate @ %g rpm/Hz)\n",
               m[7] / 100.0 * mCfg.rpmPerHz, mCfg.rpmPerHz);
        printf("bus voltage: %.1f V\n", m[2] / 10.0);
        printf("output:      %u V, %.2f A, %.1f kW, torque %.1f %%\n",
               m[3], m[4] / 100.0, m[5] / 10.0, m[6] / 10.0);
        return true;
    }

    double maxFreqHz() const { return mMaxFreqHz; }

private:
    static constexpr int kReadAttempts    = 6;
    static constexpr int kWriteAttempts   = 4;
    static constexpr long kMaxRampTicks   = 65000;  // F0.13/F0.14 range: 6500.0 s in 0.1 s units
    static constexpr int kInterFrameGapUs = 20000;

    // ponytail: 20ms inter-frame gap keeps the half-duplex bus happy
    static void gap() { usleep(kInterFrameGapUs); }

    modbus_t *mCtx     = nullptr;
    Config mCfg;
    double mMaxFreqHz  = 0;
    double mRampUnit   = 0.1;
};

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
            if (s.cmd == "run") ok = vfd.runRpm(s.a, s.b);
            else if (s.cmd == "hz") ok = vfd.runHz(std::fabs(s.a), s.b, s.a < 0);
            else if (s.cmd == "stop") ok = vfd.stop(s.b);
            else if (s.cmd == "coast") ok = vfd.coast();
            else if (s.cmd == "reset") ok = vfd.reset();
            else if (s.cmd == "wait") ok = waitMonitor(vfd, s.a, t0);
            else if (s.cmd == "write") {
                printf("write 0x%04X = %u\n", (unsigned)s.a, (unsigned)s.b);
                ok = vfd.write((uint16_t)s.a, (uint16_t)s.b);
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

#endif  // PARSER_TEST
