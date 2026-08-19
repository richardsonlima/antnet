# Builds and runs the standalone unit tests for antnet_math.h.
# These tests have no ns-2 dependency, unlike the rest of this repo (see
# ../README.md for why the full agent can't be built/tested outside an
# ns-2.34 source tree).

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -O2

.PHONY: test clean

test: test_antnet_math
	./test_antnet_math

test_antnet_math: test_antnet_math.cc ../antnet_math.h
	$(CXX) $(CXXFLAGS) -o $@ test_antnet_math.cc

clean:
	rm -f test_antnet_math
