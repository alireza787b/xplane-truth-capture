# XPlaneTruthCapture

XPlaneTruthCapture is a read-only X-Plane 11/12 dataref recorder and flight-analysis capture tool. It records frame timing, aircraft state, environment datarefs, controls, engine/rotor values, and event markers to CSV/JSON files that can be inspected offline.

It is designed for simulator integration diagnostics, XPLM plugin debugging, PX4 and ArduPilot SITL bridge validation, and reproducible flight telemetry analysis. The plugin does not control the aircraft, write actuator datarefs, change weather, or connect to an autopilot.

## Install

1. Build or download the plugin bundle named `XPlaneTruthCapture`.
2. Copy the whole folder to:

```text
X-Plane 11/Resources/plugins/XPlaneTruthCapture
X-Plane 12/Resources/plugins/XPlaneTruthCapture
```

3. Start X-Plane and open `Plugins > XPlaneTruthCapture`.

## Capture A Run

For formal px4xplane validation runs, follow `TEST_PROTOCOL.md`.

1. Load the aircraft and airport you want to test.
2. Select `Start Capture`. The menu will show `Recording Active`, and the
   optional overlay shows recording status while capture is running.
3. Fly a few simple maneuvers:
   - stationary on runway
   - taxi, accelerate, brake
   - climb and descent
   - left and right turns
   - Alia 250 or other VTOL: hover, climb, transition, fixed-wing flight, descent, landing
   - quadcopter: hover, yaw-in-place, forward/back/left/right translations, climb, descent
   - helicopter: hover, yaw, collective changes, climb, descent
4. Select `Mark Event` during important moments.
   - You can also bind the command `xplane_truth_capture/mark_event` to a key or joystick button.
   - For planned test-card events, edit `config/marker_plan.txt` and bind `xplane_truth_capture/mark_planned_event_and_advance`.
   - Generic markers stay unplanned even when a marker plan is loaded.
5. Select `Stop Capture`.
6. Zip and send the created folder from:

```text
X-Plane/Output/XPlaneTruthCapture/<run_id>
```

Use a standard `.zip` archive. Include X-Plane `Log.txt` if it was not copied automatically.

## Output Files

- `manifest.json`: X-Plane version, aircraft path, plugin path, config, and run metadata.
- `datarefs.csv`: dataref availability, type mask, writable flag, and array length.
- `frames.csv`: one row per captured frame/callback with raw values.
- `events.jsonl`: start/stop, user markers, aircraft reloads, warnings.
- `summary.json`: frame counts, dropped rows, stop reason, timing stats, pause/sim-speed counts, and automatic marker count.
- `viewer.html`: self-contained browser viewer for quick plots, frame/time x-axis selection, dark/light review, dataref search, event inspection, and selected-range export.
- `config/`: copied runtime config files used for the run.
- `tools/analyze_capture.py`: offline analyzer copied into each run folder.
- `Log.txt`: copied from the X-Plane root when accessible.

Open `viewer.html` in a browser after the run. If the browser allows local sibling-file reads, the page loads the current run automatically. Some browsers block that from `file://`; if that happens, use the file picker in the page and select the run folder or the run files.

## Custom Datarefs

The default catalog is editable:

```text
XPlaneTruthCapture/config/default_datarefs.txt
```

Format:

```text
dataref_path|group|required
```

Example:

```text
sim/flightmodel/position/q|attitude|true
```

Edit:

```text
XPlaneTruthCapture/config/datarefs.txt
```

Add one extra dataref path per line, or use the same `path|group|required` format. Restart capture after editing.

Edit:

```text
XPlaneTruthCapture/config/capture_config.ini
```

Supported settings:

- `capture_rate = every_frame | 30hz | 10hz`
- `max_array_values = 32`
- `include_default_datarefs = true | false`
- `overlay_enabled = true | false`
- `overlay_x = 24`
- `overlay_y_from_top = 48`

## Planned Markers

Edit:

```text
XPlaneTruthCapture/config/marker_plan.txt
```

Format:

```text
marker_id|name|description
```

Useful commands for keyboard or joystick binding:

- `xplane_truth_capture/mark_event`
- `xplane_truth_capture/mark_planned_event`
- `xplane_truth_capture/mark_planned_event_and_advance`
- `xplane_truth_capture/next_marker`
- `xplane_truth_capture/previous_marker`

Planned marker metadata is written to `events.jsonl` and copied into each run folder.
Use `mark_event` for an unplanned note; it does not consume the current planned marker.

## Offline Analysis

Each run folder contains the analyzer script:

```bash
python tools/analyze_capture.py /path/to/Output/XPlaneTruthCapture/<run_id>
```

It writes:

- `analysis_summary.json`
- `dataref_stats.csv`
- `derived.csv`
- `issues.jsonl`

The analyzer derives timing statistics, missing required datarefs, frame stalls, event counts, sim-time reset segments, basic derived flight columns, and per-dataref numeric stats. Array datarefs are summarized element-wise. Blank cells remain missing values; they are never converted to zero.

## Build

Install CMake and a C++17 compiler. Use the latest X-Plane SDK.

```bash
cmake -S . -B build -DXPLANE_SDK_ROOT=/path/to/SDK
cmake --build build --config Release
```

The staged plugin appears at:

```text
build/XPlaneTruthCapture
```

On this development machine, the project can also use the SDK vendored by the adjacent `px4xplane` checkout.

Windows packages built with MinGW statically link the GCC runtime so the plugin
does not require separate `libgcc_s_seh-1.dll` or `libstdc++-6.dll` files beside
`win.xpl`.

## Notes

- Missing datarefs are recorded as missing, not converted to zero.
- Effective capture rate is limited by X-Plane callback timing and simulator FPS.
- Array datarefs are dynamically sized and truncated to `max_array_values`.
- Byte-array datarefs are decoded as printable text when possible.
- Default datarefs include fixed-wing, VTOL, helicopter/rotor, weather, airdata, controls, engine, acceleration, local position, local velocity, and attitude signals.
