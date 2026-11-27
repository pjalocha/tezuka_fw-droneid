CXX ?= c++
CPPFLAGS ?=
CXXFLAGS ?= -O2 -Wall -Wextra -std=c++11
LDFLAGS ?=
LDLIBS ?= -liio -lm

all: pluto-droneid-tx dji-droneid-tx pluto-droneid-count pluto-droneid-capture pluto-droneid-scan

pluto-droneid-tx: pluto-droneid-tx.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

dji-droneid-tx: dji-droneid-tx.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

pluto-droneid-count: pluto-droneid-count.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -pthread -o $@ $<

pluto-droneid-capture: pluto-droneid-capture.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

pluto-droneid-scan: pluto-droneid-scan.cpp chirp_spectrum_detector.cpp chirp_spectrum_detector.h droneid_spectral_cfo.cpp droneid_spectral_cfo.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -pthread -o $@ pluto-droneid-scan.cpp chirp_spectrum_detector.cpp droneid_spectral_cfo.cpp $(LDLIBS)

clean:
	rm -f pluto-droneid-tx dji-droneid-tx pluto-droneid-count pluto-droneid-capture pluto-droneid-scan

.PHONY: all clean
