#include "cycle.hpp"

#include <cstdlib>
#include <sstream>

static std::string trim(const std::string &s) {
    size_t b = s.find_first_not_of(" \t\r");
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(" \t\r") - b + 1);
}

// ponytail: hand-rolled yaml subset (key: value + one "cycle:" list),
// switch to yaml-cpp if the format ever needs nesting
bool parseCycleText(std::istream &in, Config &cfg, std::vector<Step> &steps, std::string &err) {
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
        inCycle = false;  // a top-level key ends the cycle list
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

int rpmToPct(double rpm, double rpmPerHz, double maxFreqHz, bool &clamped) {
    double hz  = std::fabs(rpm) / rpmPerHz;
    int pct    = (int)std::lround(hz / maxFreqHz * 10000.0);
    clamped    = pct > 10000;
    return clamped ? 10000 : pct;
}
