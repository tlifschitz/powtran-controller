// Self-test for the cycle parser and rpm conversion — no hardware needed.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "cycle.hpp"

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
