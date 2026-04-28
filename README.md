# XPlaneTruthCapture

Read-only X-Plane diagnostic plugin for recording datarefs, frame timing, aircraft metadata, and environment state. It is built to validate simulator bridge assumptions before changing PX4, ArduPilot, or other autopilot integrations.

The plugin does not control the aircraft, write actuator datarefs, change weather, or connect to PX4.

## Install

1. Build or download the plugin bundle named `XPlaneTruthCapture`.
2. Copy the whole folder to:

```text
X-Plane 12/Resources/plugins/XPlaneTruthCapture
```

3. Start X-Plane and open `Plugins > XPlaneTruthCapture`.

## Capture A Run

1. Load the aircraft and airport you want to test.
2. Select `Start Capture`.
3. Fly a few simple maneuvers:
   - stationary on runway
   - taxi, accelerate, brake
   - climb and descent
   - left and right turns
   - hover and transition for VTOL aircraft
4. Select `Mark Event` during important moments.
5. Select `Stop Capture`.
6. Send the created folder from:

```text
X-Plane 12/Output/XPlaneTruthCapture/<run_id>
```

Include X-Plane `Log.txt` if it was not copied automatically.

## Output Files

- `manifest.json`: X-Plane version, aircraft path, plugin path, config, and run metadata.
- `datarefs.csv`: dataref availability, type mask, writable flag, and array length.
- `frames.csv`: one row per captured frame/callback with raw values.
- `events.jsonl`: start/stop, user markers, aircraft reloads, warnings.
- `summary.json`: frame counts, dropped rows, and stop reason.
- `Log.txt`: copied from the X-Plane root when accessible.

## Custom Datarefs

Edit:

```text
XPlaneTruthCapture/config/datarefs.txt
```

Add one dataref path per line. Restart capture after editing.

Edit:

```text
XPlaneTruthCapture/config/capture_config.ini
```

Supported settings:

- `capture_rate = every_frame | 30hz | 10hz`
- `max_array_values = 32`
- `include_default_datarefs = true | false`

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

## Notes

- Missing datarefs are recorded as missing, not converted to zero.
- Effective capture rate is limited by X-Plane callback timing.
- Array datarefs are dynamically sized and truncated to `max_array_values`.
- This first version records raw evidence. Analysis scripts and richer derived checks will be added after the first real capture bundle.
