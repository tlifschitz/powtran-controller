// Powtran PI150 VFD over Modbus RTU (RS485).
//
// Register map (PI150/PI500/PI550 shared protocol, verified on device):
//   0x2000 W  control: 1 fwd run, 2 rev run, 3 fwd jog, 4 rev jog,
//                      5 coast stop, 6 decel stop, 7 fault reset
//   0x1000 W  frequency setting, 0..10000 = 0..100% of max freq (F0.19)
//   0x1000 R  monitor block: setpoint echo (0.01%), running freq (0.01Hz),
//             bus V (0.1V), out V (1V), out I (0.01A), power (0.1kW),
//             torque (0.1%), rotor speed estimate (0.01Hz), DI, DO
//   0x3000 R  status: 1 fwd run, 2 rev run, 3 stop
//   0x8000 R  fault code (0 = none)
//   Fg.ii  RW parameter at 0xF0'gii' (EEPROM) or 0x0'gii' (RAM only)
#pragma once

#include <cstdint>

#include "cycle.hpp"

extern "C" {
#include <modbus/modbus.h>
}

// registers
enum : uint16_t {
    REG_FREQ_SET = 0x1000,  // write
    REG_MONITOR  = 0x1000,  // read: block documented at monitorLine()
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

class Vfd {
public:
    ~Vfd();

    bool open(const Config &cfg);
    bool read(uint16_t addr, uint16_t *dst, int n = 1);
    bool write(uint16_t addr, uint16_t val);
    bool loadScaling();
    bool setRamp(double sec, bool accel, bool decel);
    bool runRpm(double rpm, double ramp);
    bool runHz(double hz, double ramp, bool reverse);
    bool stop(double ramp);
    bool coast();
    bool reset();
    bool monitorLine(double t);
    bool status();

private:
    static constexpr int kReadAttempts    = 6;
    static constexpr int kWriteAttempts   = 4;
    static constexpr long kMaxRampTicks   = 65000;  // F0.13/F0.14 range: 6500.0 s in 0.1 s units
    static constexpr int kInterFrameGapUs = 20000;

    // ponytail: 20ms inter-frame gap keeps the half-duplex bus happy
    static void gap();

    modbus_t *mCtx    = nullptr;
    Config mCfg;
    double mMaxFreqHz = 0;
    double mRampUnit  = 0.1;
};
