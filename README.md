# spatial-audio

Head-tracked spatial audio for Sony earbuds on Linux (PipeWire) — the equivalent of Android's / iOS's
"spatial audio with head tracking". The sound stays put in the room while you turn your head, nod or tilt.

It runs as a transparent **PipeWire filter in front of your earbuds**, the way Android's spatializer does: the earbuds
stay your default output, so volume keys, codec choice (LDAC/AAC) and reconnects behave exactly as before. Nothing
shows up as a separate sound device you have to select.

> **Status:** works on the Sony **WF-1000XM5** (Ubuntu 26.04, PipeWire 1.6, WirePlumber 0.5, GNOME on Wayland). Any
> Sony headset that SonyTrackerLinux supports (WH-1000XM5/XM6, WF-1000XM6, LinkBuds, ULT WEAR ...) is detected
> automatically, but only the WF-1000XM5 has been tested — reports welcome.

## Features

- **Head tracking from the earbuds' own sensor** (Android Head Tracker HID protocol), no phone needed.
- **Ambisonics rendering**, like phone spatializers: a measured HRTF (SADIE II KU100) is fitted once into 16
  spherical-harmonic channels (order 3). At runtime the two virtual speakers are encoded at their current direction
  in the head's frame and filtered with *fixed* filters, so head movement only changes smooth encoding gains; no
  filter is ever switched or interpolated between measurements. Up/down and tilt work. ~7 % of one CPU core while
  playing. The centred sound keeps the level and tone of the original stereo, and a limiter keeps turns from clipping.
- **Android-style auto-recenter**: after your head has been still for ~2 s, wherever you face becomes the new front.
- **Click-free**: smooth fractional delays, rotation (quaternion) smoothing, and directions passed as vectors, not angles: no
  snaps at 180°, no flips when looking straight down.
- **Only active when it makes sense**: available when the selected output is stereo (A2DP) Sony earbuds; not for
  hands-free calls or speakers. It follows connect/disconnect automatically.
- **Event driven**: no polling; ~0 % CPU when nothing is playing. Runs as a systemd user service.
- Optional **GNOME Quick Settings tile** (on/off, strength slider).

## How it works

```
 earbuds ──Bluetooth HID──▶ sony-tracker ──UDP (yaw/pitch/roll)──▶ head-track ──PipeWire params──┐
                                                                                                  ▼
 apps ─▶ [ spatial filter (PipeWire filter-chain, WirePlumber "smart filter") ] ─▶ earbuds sink (LDAC/AAC)
              ▲
 spatial-daemon: watches PipeWire, Bluetooth, udev and the on/off switch; starts/stops the pieces above
```

See [docs/architecture.md](docs/architecture.md) for the details and the design decisions.

## Requirements

- Linux with **PipeWire ≥ 1.0** and **WirePlumber ≥ 0.5** (smart filters), BlueZ, systemd (user session).
- Sony earbuds paired and set to the normal stereo (A2DP) profile.
- To build: a C++17 compiler, `make`, `pkg-config` and the development packages below.

```bash
sudo apt install build-essential pkg-config libpipewire-0.3-dev libmysofa-dev libglib2.0-dev libudev-dev
```

## Install

```bash
git clone --recurse-submodules https://github.com/ArifBryan/spatial-audio.git
cd spatial-audio
make                      # builds everything into build/
sudo make install-udev    # lets your user read the earbuds' head-tracker sensor (one device ID only)
make install              # user service + `spatial` command (in ~/.local/bin)
spatial on
```

Optional pieces:

```bash
make hrtf                 # downloads the SADIE II KU100 HRTF (38 MB) and builds the Ambisonics filters (required)
make install-extension    # GNOME Quick Settings tile; log out and in, then: gnome-extensions enable spatial-audio@local
make test                 # self-checks (renderer plugin, limiter, gimbal-lock handling)
```

Supported devices are detected by capability, not by name: a Sony Bluetooth device (vendor 054C) that offers the
head-tracker HID service, the same test SonyTrackerLinux uses. The udev rule covers every Sony Bluetooth HID device.

## Use

```bash
spatial on | off | toggle | status
spatial force on|off          # TESTING: skip the hardware checks, attach the filter to the selected output
spatial strength 100          # how much of your head movement the sound stage follows (0-100)
spatial lookahead 0           # optional look-ahead in ms (0 = off)
spatial doctor                # print diagnostics
```

`spatial on` is remembered across logins. The effect only runs while compatible earbuds are the selected output.

## Troubleshooting

See [docs/troubleshooting.md](docs/troubleshooting.md). The most common causes: the earbuds slipped into the
hands-free profile (switch back to A2DP), the head-tracker sensor profile is missing after a reconnect (the daemon
asks for it again automatically), or a browser kept a stale audio connection (restart the browser's audio process).

## Credits and licences

- This project: [MIT](LICENSE).
- [SonyTrackerLinux](https://github.com/kdani3/SonyTrackerLinux) (MIT) reads the earbuds' sensor; it is a Linux port
  of [sony-head-tracker](https://github.com/NicholasSlattery/sony-head-tracker) by Nicholas Slattery. Included as a
  git submodule.
- HRTF data is **not** distributed here: `make hrtf` downloads the
  [SADIE II database](https://www.york.ac.uk/sadie-project/database.html) (University of York, Apache-2.0) from Zenodo.
- Uses [libmysofa](https://github.com/hoene/libmysofa) (BSD) to read SOFA files, and PipeWire / WirePlumber.
- Not affiliated with or endorsed by Sony.
