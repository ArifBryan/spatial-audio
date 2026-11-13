# Architecture and design notes

## Components

| Program | Language | Role |
|---|---|---|
| `spatial-daemon` | C++ (GLib, libpipewire, libudev) | systemd user service. Watches the PipeWire graph, Bluetooth (BlueZ D-Bus), the sensor device (udev) and the on/off switch (file). Starts and stops the filter, the helper and the tracker. Event driven; no timers. |
| `head-track` | C++ (libpipewire) | Reads head pose from UDP, recenters, smooths, converts to filter settings and sends them straight to the filter node. |
| `sony-tracker` | C (third party) | Reads the earbuds' Android-Head-Tracker HID sensor and emits OpenTrack-style UDP packets. |
| `libspatial-dsp.so` | C++ (LADSPA) | The `ambirot` Ambisonics renderer (see below). |
| `spatial` | C++ | Command line. Writes small files the daemon reacts to. |
| `spatial-tools` | C++ (libmysofa) | Offline: fits an HRTF into Ambisonic channels and writes the filter data and the PipeWire graph (`gen-ambi`). |
| GNOME extension | JavaScript | Optional Quick Settings tile (reads `$XDG_RUNTIME_DIR/spatial-audio/state.json`). |

## The filter is a WirePlumber *smart filter*

The spatial graph is a `libpipewire-module-filter-chain` whose capture node declares `filter.smart = true` and
`filter.smart.target = { bluez5.sink-loopback-target = true }`. WirePlumber then slots it between application streams
and the earbuds' sink automatically, so the earbuds remain the default output. Notes learned the hard way:

- Bluetooth sinks come as a *loopback filter* (`bluez_output.<MAC>`) in front of the real sink
  (`Audio/Sink/Internal`). Filters cannot target filters, so ours targets the real sink, and a metadata hint
  (`filter.smart.after`) keeps our filter *ahead* of the Bluetooth loopback. The daemon re-publishes that hint
  whenever the loopback node is recreated.
- The filter node must stay a plain `Audio/Sink`. Marking it `Audio/Sink/Internal` hides it from sound settings but
  breaks PulseAudio clients such as Chrome, which look up the sink their stream is attached to.
- The node's own volume is pinned to 100 % by the daemon, so only the earbuds' real volume matters.

## Head pose handling

- Pose is kept as a **quaternion**. Smoothing and look-ahead operate on the rotation. Smoothing yaw/pitch/roll
  separately snaps at ±180° and near "looking straight down" (gimbal lock).
- Recentering is stillness based (like Android): while the head moves the centre stays; after ~2 s of stillness the
  centre glides to the current heading. Only the *heading* is recentred, and not while looking steeply up/down where
  yaw is meaningless.

## Switching on and off

`spatial off` makes the effect fade to plain stereo within ~150 ms, stops the tracker and helper, and removes the
filter as soon as PipeWire reports nothing is playing through it. Streams are never moved while audio is playing
(some apps, e.g. browsers, pause when their stream is moved).

## Tests

`make test` checks: the head-pose pipeline through a
gimbal-lock look-down; the renderer (plain stereo at Mix 0, centred level within 3 dB of the input, output never above full scale, no clicks when a speaker passes over the pole, ears differ when the head turns).

## Ambisonics renderer

Like the spatializers in phones, but for two virtual speakers. `spatial-tools gen-ambi` fits the measured HRTF (minimum-phase
filters, all 8,802 directions) into 16 real spherical-harmonic channels (order 3, ACN/SN3D; `src/sh.h` is shared with the
plugin) by least squares, fits the interaural time difference in the same basis, and applies a small diffuse-field
correction for the high-frequency energy that truncating at order 3 removes. A fixed, centre-matched correction then divides out the
rendered response of the ±30° pair (power-domain smoothed, about a third of an octave), so a centred sound has the level and tone of the
original stereo. The `ambirot` plugin takes eight inputs (PipeWire's 7.1 order) and encodes each virtual
speaker at its head-frame direction (gains ramp linearly across each 64-sample block), delays each ear with a smooth
fractional delay, and filters the 16 channels with the fixed filters. Directions are passed to the plugin as unit vectors (never
azimuth/elevation), so there is no singularity when a speaker passes over the head. A block limiter (ceiling 0.95) guards against
clipping when the near ear gets louder, and a `Mix` control fades to plain stereo for switching off. The strength setting scales the
head rotation (0 = fixed stereo, 100 = the sound stage follows exactly). Nothing is switched or interpolated between
measurements while the head moves. Order 3 captured the filters with a relative error of about -9 dB; higher orders or
magnitude-least-squares fitting would be the next step for sharper localisation.

## Surround

The filter sink is 8 channels (`FL FR FC LFE RL RR SL SR`, `channelmix.upmix = false`, so a stereo stream fills only FL and FR). The
plugin keeps the speaker layout itself: FL/FR at ±30°, FC at 0°, SL/SR at ±90°, RL/RR at ±135° (a 5.1 stream's rears land there too),
centre and surrounds at −3 dB, LFE added to both ears untouched. The head rotation reaches the plugin as a quaternion and the plugin
rotates each speaker into the head frame. All speakers share the same 16 fixed filters, because they are summed in the
Ambisonic domain before filtering, so each extra speaker only costs its encoding (a delay and 16 gains per ear). A speaker whose input
has been silent for a few milliseconds is skipped. With Mix at 0 the plugin outputs the standard stereo downmix of its eight inputs.
The tone calibration (`spatial-tools calibrate`) is measured with the front pair only.
