// Cycle-file model and parser — hardware-free, covered by cycle-test.
#pragma once

#include <cmath>
#include <istream>
#include <string>
#include <vector>

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

bool parseCycleText(std::istream &in, Config &cfg, std::vector<Step> &steps, std::string &err);

// rpm -> 0x1000 setpoint in 0.01% of max freq, 10000 = 100%
// (always positive; direction is chosen by the run command)
int rpmToPct(double rpm, double rpmPerHz, double maxFreqHz, bool &clamped);
