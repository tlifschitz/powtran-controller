#include "vfd.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>

extern "C" {
#include <unistd.h>
}

Vfd::~Vfd() {
    if (mCtx) {
        modbus_close(mCtx);
        modbus_free(mCtx);
    }
}

bool Vfd::open(const Config &cfg) {
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

void Vfd::gap() { usleep(kInterFrameGapUs); }

// VFD switching noise (EMI) puts junk bytes on the bus while the motor
// runs: flush before every attempt and retry a few times. Reads get more
// attempts than writes because telemetry polls hit the noise most often.
bool Vfd::read(uint16_t addr, uint16_t *dst, int n) {
    for (int t = 0; t < kReadAttempts; ++t) {
        modbus_flush(mCtx);
        if (modbus_read_registers(mCtx, addr, n, dst) == n) { gap(); return true; }
        gap();
    }
    fprintf(stderr, "read 0x%04X: %s\n", addr, modbus_strerror(errno));
    return false;
}

bool Vfd::write(uint16_t addr, uint16_t val) {
    for (int t = 0; t < kWriteAttempts; ++t) {
        modbus_flush(mCtx);
        if (modbus_write_register(mCtx, addr, val) == 1) { gap(); return true; }
        gap();
    }
    fprintf(stderr, "write 0x%04X=%u: %s\n", addr, val, modbus_strerror(errno));
    return false;
}

// reads scaling params from the VFD (once)
bool Vfd::loadScaling() {
    if (mMaxFreqHz > 0) return true;
    uint16_t v;
    if (!read(EE_F0_19, &v)) return false;
    mMaxFreqHz = v / 100.0;
    if (!read(EE_F0_15, &v)) return false;
    mRampUnit = (v == 0) ? 1.0 : (v == 1) ? 0.1 : 0.01;
    printf("vfd: max freq %.2f Hz, ramp unit %.2g s\n", mMaxFreqHz, mRampUnit);
    return true;
}

bool Vfd::setRamp(double sec, bool accel, bool decel) {
    long ticks = std::lround(sec / mRampUnit);
    if (ticks > kMaxRampTicks) ticks = kMaxRampTicks;
    if (accel && !write(RAM_ACCEL, (uint16_t)ticks)) return false;
    if (decel && !write(RAM_DECEL, (uint16_t)ticks)) return false;
    return true;
}

bool Vfd::runRpm(double rpm, double ramp) {
    if (!loadScaling()) return false;
    if (mCfg.maxRpm > 0 && std::fabs(rpm) > mCfg.maxRpm) {
        printf("clamp: %g -> %g rpm (max_rpm)\n", rpm, mCfg.maxRpm);
        rpm = rpm < 0 ? -mCfg.maxRpm : mCfg.maxRpm;
    }
    return runHz(std::fabs(rpm) / mCfg.rpmPerHz, ramp, rpm < 0);
}

bool Vfd::runHz(double hz, double ramp, bool reverse) {
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

bool Vfd::stop(double ramp) {
    if (!std::isnan(ramp)) {
        if (!loadScaling() || !setRamp(ramp, false, true)) return false;
    }
    printf("stop (decel)\n");
    return write(REG_CONTROL, CMD_STOP);
}

bool Vfd::coast() { printf("stop (coast)\n"); return write(REG_CONTROL, CMD_COAST); }
bool Vfd::reset() { printf("fault reset\n"); return write(REG_CONTROL, CMD_RESET); }

// monitor block, verified live: [0] setpoint echo 0.01%, [1] running
// freq 0.01Hz, [2] bus 0.1V, [3] out V, [4] out 0.01A, [5] power 0.1kW,
// [6] torque 0.1%, [7] rotor speed estimate 0.01Hz (slip-adjusted)
bool Vfd::monitorLine(double t) {
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

bool Vfd::status() {
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
