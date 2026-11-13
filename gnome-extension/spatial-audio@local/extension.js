import GObject from 'gi://GObject';
import Gio from 'gi://Gio';
import GLib from 'gi://GLib';
import St from 'gi://St';

import {Extension} from 'resource:///org/gnome/shell/extensions/extension.js';
import * as Main from 'resource:///org/gnome/shell/ui/main.js';
import * as QuickSettings from 'resource:///org/gnome/shell/ui/quickSettings.js';
import * as PopupMenu from 'resource:///org/gnome/shell/ui/popupMenu.js';
import {Slider} from 'resource:///org/gnome/shell/ui/slider.js';

const SPATIAL = GLib.build_filenamev([GLib.get_home_dir(), '.local', 'bin', 'spatial']);
const STATE_DIR = GLib.build_filenamev([GLib.get_user_runtime_dir(), 'spatial-audio']);
const STATE_FILE = GLib.build_filenamev([STATE_DIR, 'state.json']);
const FILTER_DESCRIPTION = 'Spatial Head-Tracked';   // the filter's name in GNOME's sound output list

// Run the `spatial` command; resolves with its stdout.
function run(args) {
    return new Promise(resolve => {
        try {
            const proc = Gio.Subprocess.new([SPATIAL, ...args],
                Gio.SubprocessFlags.STDOUT_PIPE | Gio.SubprocessFlags.STDERR_SILENCE);
            proc.communicate_utf8_async(null, null, (p, res) => {
                try { resolve(p.communicate_utf8_finish(res)[1] ?? ''); } catch (e) { resolve(''); }
            });
        } catch (e) { resolve(''); }
    });
}

const SpatialToggle = GObject.registerClass(
class SpatialToggle extends QuickSettings.QuickMenuToggle {
    constructor(icon) {
        super({
            title: 'Spatial Audio',
            gicon: icon,
            toggleMode: true,
        });
        this._available = false;
        this._dragging = false;
        this.connect('clicked', () => this._onClicked());
        this._icon = icon;
        this._buildMenu();
        this._apply(false, false);
    }

    _buildMenu() {
        this.menu.setHeader(this._icon, 'Spatial Audio');

        // Strength slider (0-100): applied when released, or immediately for keyboard / scroll changes
        const item = new PopupMenu.PopupBaseMenuItem({activate: false});
        const label = new St.Label({text: 'Strength', y_align: 2});
        this._slider = new Slider(0.55);
        this._slider.x_expand = true;
        const apply = () => run(['strength', String(Math.round(this._slider.value * 100))]);
        this._slider.connect('drag-begin', () => { this._dragging = true; });
        this._slider.connect('drag-end', () => { this._dragging = false; apply(); });
        this._slider.connect('notify::value', () => { if (!this._dragging && this.menu.isOpen) apply(); });
        item.add_child(label);
        item.add_child(this._slider);
        this.menu.addMenuItem(item);

        // show the stored value whenever the menu opens
        this.menu.connect('open-state-changed', async (_m, open) => {
            if (!open) return;
            const m = (await run(['strength'])).match(/strength:\s*(\d+)/);
            if (m) this._slider.value = Math.min(100, Number(m[1])) / 100;
        });
    }

    async _onClicked() {
        if (!this._available) return;               // unavailable: ignore everything
        await run([this.checked ? 'on' : 'off']);
        // the daemon's state file change updates us; nothing to poll
    }

    // on: switch is on; available: compatible stereo earbuds are the selected output
    setState(on, available) { this._apply(on, available); }

    _apply(on, available) {
        this._available = available;
        // Fully inert when unavailable: not clickable, not focusable, no hover, no submenu arrow, dimmed.
        this.reactive = available;
        this.can_focus = available;
        this.track_hover = available;
        this.menuEnabled = available;
        this.toggleMode = available;
        this.opacity = available ? 255 : 110;
        if (available) this.remove_style_pseudo_class('insensitive');
        else this.add_style_pseudo_class('insensitive');
        this.checked = available && on;
        this.subtitle = available ? (on ? 'On' : 'Off') : 'Not available';
    }
});

const SpatialIndicator = GObject.registerClass(
class SpatialIndicator extends QuickSettings.SystemIndicator {
    constructor(icon) {
        super();
        // No icon in the top bar: the Quick Settings tile alone shows the state.
        this._toggle = new SpatialToggle(icon);
        this.quickSettingsItems.push(this._toggle);

        // Event driven: the daemon rewrites state.json whenever anything relevant changes.
        GLib.mkdir_with_parents(STATE_DIR, 0o700);
        this._monitor = Gio.File.new_for_path(STATE_DIR).monitor_directory(Gio.FileMonitorFlags.NONE, null);
        this._monitorId = this._monitor.connect('changed', () => this._readState());
        this._readState();
    }

    _readState() {
        Gio.File.new_for_path(STATE_FILE).load_contents_async(null, (file, res) => {
            let on = false, available = false;
            try {
                const [, bytes] = file.load_contents_finish(res);
                const s = JSON.parse(new TextDecoder().decode(bytes));
                on = !!s.enabled; available = !!s.compatible;
            } catch (e) { /* no daemon / no state yet: unavailable */ }
            this._toggle.setState(on, available);
        });
    }

    destroy() {
        this._monitor?.disconnect(this._monitorId);
        this._monitor?.cancel();
        this._monitor = null;
        this.quickSettingsItems.forEach(i => i.destroy());
        super.destroy();
    }
});

export default class SpatialAudioExtension extends Extension {
    enable() {
        const icon = Gio.icon_new_for_string(GLib.build_filenamev([this.path, 'icons', 'spatial-audio-symbolic.svg']));
        this._indicator = new SpatialIndicator(icon);
        Main.panel.statusArea.quickSettings.addExternalIndicator(this._indicator);
        this._hideFilterFromOutputList();
    }

    disable() {
        this._showFilterInOutputList();
        this._indicator?.destroy();
        this._indicator = null;
    }

    // The filter is an implementation detail, not a place to send audio: keep it out of the Quick Settings
    // "Sound Output" list. (It stays a normal sink underneath, which Chrome and other apps need.)
    _hideFilterFromOutputList() {
        const out = Main.panel.statusArea.quickSettings._volumeOutput?._output;
        if (!out || this._origAddDevice) return;
        const isFilter = id => out._lookupDevice(id)?.description === FILTER_DESCRIPTION;
        this._origAddDevice = out._addDevice;
        out._addDevice = function (id) {
            if (isFilter(id)) return;
            return this.constructor.prototype._addDevice.call(this, id);
        };
        for (const id of [...out._deviceItems.keys()])
            if (isFilter(id)) out._removeDevice(id);
        this._output = out;
    }

    _showFilterInOutputList() {
        if (!this._output) return;
        delete this._output._addDevice;           // fall back to the class method
        this._output = null; this._origAddDevice = null;
    }
}
