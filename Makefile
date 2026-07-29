# Builds powtran-ctl natively (x86). For an ARM target, point CROSS_CXX at
# your cross compiler (with --sysroot providing libmodbus):
#   make cross CROSS_CXX="arm-linux-gnueabihf-g++ --sysroot=/path/to/sysroot"
CXXFLAGS = -O2 -Wall -Wextra -std=c++17

all: x86

x86: build/x86/powtran-ctl
build/x86/powtran-ctl: src/powtran-ctl.cpp
	mkdir -p $(@D)
	g++ $(CXXFLAGS) -o $@ $< -lmodbus

cross: build/cross/powtran-ctl
build/cross/powtran-ctl: src/powtran-ctl.cpp
	mkdir -p $(@D)
	$(CROSS_CXX) $(CXXFLAGS) -o $@ $< -lmodbus

check:
	mkdir -p build
	g++ $(CXXFLAGS) -DPARSER_TEST -o build/parser-test src/powtran-ctl.cpp
	./build/parser-test

clean:
	rm -rf build

.PHONY: all x86 cross check clean
