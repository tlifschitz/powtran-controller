# Builds powtran-ctl natively (x86). For an ARM target, point CROSS_CXX at
# your cross compiler (with --sysroot providing libmodbus):
#   make cross CROSS_CXX="arm-linux-gnueabihf-g++ --sysroot=/path/to/sysroot"
CXXFLAGS = -O2 -Wall -Wextra -std=c++17

SRC = src/main.cpp src/vfd.cpp src/cycle.cpp
HDR = src/cycle.hpp src/vfd.hpp

all: x86

x86: build/x86/powtran-ctl
build/x86/powtran-ctl: $(SRC) $(HDR)
	mkdir -p $(@D)
	g++ $(CXXFLAGS) -o $@ $(SRC) -lmodbus

cross: build/cross/powtran-ctl
build/cross/powtran-ctl: $(SRC) $(HDR)
	mkdir -p $(@D)
	$(CROSS_CXX) $(CXXFLAGS) -o $@ $(SRC) -lmodbus

check:
	mkdir -p build
	g++ $(CXXFLAGS) -o build/parser-test src/cycle-test.cpp src/cycle.cpp
	./build/parser-test

clean:
	rm -rf build

.PHONY: all x86 cross check clean
