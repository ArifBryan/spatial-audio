# Troubleshooting

Start with `spatial doctor`. Logs: `journalctl --user -u spatial-audio` and `$XDG_RUNTIME_DIR/spatial-audio/*.log`.

**"Spatial: ON (waiting for compatible earbuds as the output)"** — the selected output is not stereo Sony earbuds.
Check the earbuds are connected, the output is the earbuds, and the Bluetooth profile is *A2DP* (not "Headset /
hands-free"). In hands-free mode audio is mono and low quality; switch the profile back in Sound settings.

**Effect doesn't follow the head** (`tracker: waiting for sensor`) — the earbuds' sensor (HID) profile is not
connected. The daemon asks BlueZ for it after each connect; manually:
`busctl --system call org.bluez /org/bluez/hci0/dev_XX_XX_XX_XX_XX_XX org.bluez.Device1 ConnectProfile s 00001124-0000-1000-8000-00805f9b34fb`.
Also check the udev rule is installed (`sudo make install-udev`) so your user can read `/dev/hidrawN`.

**Tracker says "no report within 2000 ms"** — the sensor stalled (it can stop after an earbud is taken out). The daemon
restarts the tracker automatically.

**A browser plays silence or spins after switching things** — its audio connection went stale. Restart the browser's
audio helper process (for Chrome: `kill $(pgrep -f audio.mojom.AudioService)`) or the browser.

**Sound is quiet after a restart** — `spatial doctor`; the daemon pins the filter's volume to 100 %, so the earbuds'
volume is the only one that counts.

**Does not engage at all / apps bypass the filter** — it needs WirePlumber ≥ 0.5 smart-filter support. After upgrading
or heavy experimentation, `systemctl --user restart wireplumber` clears stale state.

**Different Sony model** — detected automatically if it exposes the head-tracker HID service. Re-run `sudo make install-udev` so the udev rule covers its product ID, then reconnect it.
