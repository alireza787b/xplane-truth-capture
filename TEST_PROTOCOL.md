# XPlaneTruthCapture Controlled Test Protocol

Use the latest release build and send the complete run folder as a standard `.zip`.

## Before Flying

- Install the whole `XPlaneTruthCapture` folder into `X-Plane/Resources/plugins/`.
- Start X-Plane, load the aircraft, and wait until the aircraft is stable.
- If possible, use clear weather and low/no wind for the first controlled run.
- Edit `XPlaneTruthCapture/config/marker_plan.txt` if you want custom maneuver names.
- Bind `xplane_truth_capture/mark_planned_event_and_advance` to a keyboard key or joystick button.
- Bind `xplane_truth_capture/mark_event` as a backup generic marker if possible.
- Do not reload or change aircraft during a capture.

## Capture Steps

1. Start capture from `Plugins > XPlaneTruthCapture > Start Capture`.
2. Press the planned marker key before and after each maneuver.
3. Keep each marked maneuver simple and separated by 5-10 seconds of steady flight when possible.
4. Stop capture from `Plugins > XPlaneTruthCapture > Stop Capture`.
5. Zip the created folder from `X-Plane/Output/XPlaneTruthCapture/<run_id>/`.

## Fixed-Wing Baseline

Recommended aircraft: C172 or another stable fixed-wing aircraft.

- stationary on runway for 15 seconds
- forward throttle acceleration
- brake or idle deceleration
- takeoff and steady climb
- level flight
- gentle left turn
- gentle right turn
- descent
- landing and stop

## Helicopter or Quadcopter

The hover section is important.

- stationary on ground for 15 seconds
- takeoff to hover
- steady hover for 20 seconds
- yaw left in place
- yaw right in place
- translate forward
- translate backward
- translate right
- translate left
- climb
- descend
- landing and stop

## Alia 250 or Other VTOL

- stationary on ground
- hover for 20 seconds
- yaw in hover
- forward transition
- fixed-wing cruise
- left and right turns
- back transition
- vertical landing and stop

## Notes

- If the aircraft crashes, stop the capture and send it anyway, but label it as a crash run.
- If X-Plane reloads the aircraft during capture, stop and start a new capture.
- Send the `.zip` plus any short notes about aircraft, X-Plane version, weather, and the maneuver order.
