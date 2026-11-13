# Head-tracked spatial audio for Sony earbuds (PipeWire).  Build, fetch the HRTF data, install.
#
#   make              build everything into build/
#   make hrtf         (optional) download the SADIE II KU100 HRTF and generate the "bank" mode data
#   make test         run the self-checks
#   make install      install the user service, the `spatial` command and (separately) the GNOME extension
#   sudo make install-udev    let your user read the earbuds' head-tracker sensor
#   make uninstall

CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wno-missing-field-initializers   # PipeWire event tables are filled partially on purpose
BUILD    := build
PW_CFLAGS   := $(shell pkg-config --cflags libpipewire-0.3)
PW_LIBS     := $(shell pkg-config --libs libpipewire-0.3)
GLIB_CFLAGS := $(shell pkg-config --cflags glib-2.0 gio-2.0 gio-unix-2.0 libudev)
GLIB_LIBS   := $(shell pkg-config --libs glib-2.0 gio-2.0 gio-unix-2.0 libudev)

TRACKER  := third_party/SonyTrackerLinux/sony-tracker
HRTF_URL := https://zenodo.org/records/12092466/files/D1_HRIR_SOFA.zip?download=1
HRTF_SHA := 366321fa78f211bacc0ec6bea96701625b196a5db54ec16748b0eab0b9705f75
EXT_NAME := spatial-audio@local
EXT_DIR  := $(HOME)/.local/share/gnome-shell/extensions/$(EXT_NAME)

all: $(BUILD)/head-track $(BUILD)/spatial-daemon $(BUILD)/spatial $(BUILD)/spatial-tools \
     $(BUILD)/libspatial-dsp.so $(TRACKER)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/head-track: src/head-track.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(PW_CFLAGS) -o $@ $< $(PW_LIBS)

$(BUILD)/spatial-daemon: src/spatial-daemon.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(PW_CFLAGS) $(GLIB_CFLAGS) -o $@ $< $(PW_LIBS) $(GLIB_LIBS)

$(BUILD)/spatial: src/spatial.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BUILD)/spatial-tools: src/spatial-tools.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $< -lmysofa -ldl

$(BUILD)/libspatial-dsp.so: src/dsp.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -O3 -shared -fPIC -o $@ $< -ldl

$(TRACKER):
	$(MAKE) -C third_party/SonyTrackerLinux

# ---- HRTF data: downloads 38 MB from Zenodo, Apache-2.0, SADIE II database, University of York)
hrtf: $(BUILD)/spatial-tools $(BUILD)/libspatial-dsp.so
	mkdir -p data/downloads data/hrtf
	[ -f data/downloads/D1_HRIR_SOFA.zip ] || curl -fL -o data/downloads/D1_HRIR_SOFA.zip '$(HRTF_URL)'
	echo "$(HRTF_SHA)  data/downloads/D1_HRIR_SOFA.zip" | sha256sum -c -
	unzip -oq data/downloads/D1_HRIR_SOFA.zip -d data/hrtf
	$(BUILD)/spatial-tools gen-ambi
	for i in 1 2 3; do $(BUILD)/spatial-tools calibrate; done

# ---- tests
TESTS := $(BUILD)/test-dsp $(BUILD)/test-gimbal
$(BUILD)/test-dsp: tests/dsp.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $< -ldl
$(BUILD)/test-gimbal: tests/gimbal.cpp src/head-track.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -Wno-unused-function $(PW_CFLAGS) -o $@ $< $(PW_LIBS)


test: $(TESTS) $(BUILD)/libspatial-dsp.so
	$(BUILD)/test-dsp $(BUILD)/libspatial-dsp.so
	$(BUILD)/test-gimbal

# ---- install (user level)
install: all
	mkdir -p $(HOME)/.local/bin $(HOME)/.config/systemd/user
	ln -sf $(CURDIR)/$(BUILD)/spatial $(HOME)/.local/bin/spatial
	sed 's|@BUILD@|$(CURDIR)/$(BUILD)|' packaging/spatial-audio.service.in > $(HOME)/.config/systemd/user/spatial-audio.service
	systemctl --user daemon-reload
	@echo "Installed. Turn it on with:  spatial on   (see README for the optional GNOME tile and the udev rule)"

install-extension:
	mkdir -p $(EXT_DIR)
	cp -r gnome-extension/$(EXT_NAME)/. $(EXT_DIR)/
	@echo "Log out and in (Wayland), then: gnome-extensions enable $(EXT_NAME)"

install-udev:
	install -m 0644 packaging/71-sony-head-tracker.rules /etc/udev/rules.d/71-sony-head-tracker.rules
	udevadm control --reload
	udevadm trigger --subsystem-match=hidraw

uninstall:
	-systemctl --user disable --now spatial-audio.service
	rm -f $(HOME)/.config/systemd/user/spatial-audio.service $(HOME)/.local/bin/spatial
	rm -rf $(EXT_DIR)
	systemctl --user daemon-reload
	@echo "Remove the udev rule with: sudo rm /etc/udev/rules.d/71-sony-head-tracker.rules && sudo udevadm control --reload"

clean:
	rm -rf $(BUILD)
	-$(MAKE) -C third_party/SonyTrackerLinux clean

.PHONY: all hrtf test install install-extension install-udev uninstall clean
