# M5StickC Plus Attitude HUD

This document describes the current heads-up display proof of concept in
`src/main.cpp`. It is intended as project memory for future development and as
a guide for other developers picking up the sketch.

The display is an Elite-inspired attitude gauge for the M5StickC Plus. It uses
the IMU to show pitch and roll relative to a user-defined reference attitude,
with visual, LED, and buzzer feedback for level, warning, alarm, and inverted
states.

## Goals

- Show roll and pitch in a compact, readable instrument-style UI.
- Preserve the rotating cube as an optional reference view.
- Support a two-device controller/sensor arrangement where the display controller
  receives attitude data from a remote sensor node over UART.
- Make the lower plot feel like an 8-bit attitude gauge: stark geometry,
  high-contrast colour states, and simple directional cues.
- Keep the HUD loop responsive. All user input, LED pulses, buzzer patterns,
  alarm flashing, and sound toggle tones are asynchronous and based on
  `millis()`.
- Treat startup, or a Button A reference reset, as the nominal level attitude.

## Hardware And Display Setup

- Target device: M5StickC Plus.
- Display orientation: portrait, `135 x 240`.
- Sprite buffer: `TFT_eSprite Disbuff`, pushed using `Displaybuff()`.
- Screen brightness is currently set in `setup()` with:

```cpp
M5.Axp.ScreenBreath(100);
```

- The HUD clears each frame with `DISPLAY_BACKGROUND_COLOR`, currently
  `TFT_NAVY`.
- `showCube` controls whether the cube reference may be drawn in circle mode. It
  is currently `true`.
- The HUD starts in ellipse mode. In ellipse mode the cube is never prepared,
  calculated, or drawn. When Button A toggles into circle mode, the cube is drawn
  only if `showCube` was enabled at compile time.
- The role marker is a single white character:
  - `S` for sensor node.
  - `C` for display controller.
  It is drawn immediately after the navy background clear, before the plot and
  spot, so attitude graphics can paint over it.

## Controller/Sensor UART Mode

The sketch can run in either sensor-node mode or display-controller mode. This is
controlled by one flag near the top of `src/main.cpp`:

```cpp
static constexpr bool DEVICE_IS_SENSOR_NODE = false;
```

### Sensor Node

Set `DEVICE_IS_SENSOR_NODE` to `true` for the device mounted on the object being
monitored.

- Reads the local IMU.
- Owns the reference attitude used as "level to earth".
- Button A short hold/release resets that reference on the sensor node.
- Sends referenced pitch, referenced roll, and inverted state over UART.
- Still draws the local HUD, drives its own LED/buzzer, and responds to all
  local buttons.
- Shows `S` as its static role marker.

### Display Controller

Set `DEVICE_IS_SENSOR_NODE` to `false` for the display device.

- Does not read the local IMU in the HUD loop.
- Receives referenced pitch, referenced roll, and inverted state from the sensor
  node.
- Drives the display, LED, buzzer, alarm logic, limit cycling, sound toggle, and
  circle/ellipse mode locally.
- Button A short hold/release does not reset attitude on the controller because
  the reference lives on the sensor node.
- Button A long hold still toggles circle/ellipse mode.
- Shows `C` as its static role marker.

### UART Link

The IMU link uses a separate ESP32 UART:

```cpp
HardwareSerial ImuLinkSerial(2);
```

Current link settings:

- Baud rate: `9600`.
- Controller RX / sensor TX pin: GPIO `32`.
- Controller TX / sensor RX pin: GPIO `33`.
- Data format: `SERIAL_8N1`.

This means two M5StickC Plus Grove ports can be connected straight-through:
GPIO `32` to GPIO `32`, GPIO `33` to GPIO `33`, and ground to ground. The role
flag swaps RX/TX internally so the sensor node transmits on the pin that the
display controller receives.

The existing USB `Serial` object is still available for local debug messages, but
it is not used for the IMU data link.

### UART Frame Format

The sensor node sends compact 8-byte binary frames. Pitch and roll are sent as
signed centidegrees so the display still gets 0.01 degree resolution without
floating-point text over the UART link.

| Byte(s) | Field | Meaning |
| ---: | --- | --- |
| `0` | Magic 0 | `0xA5` |
| `1` | Magic 1 | `0x5A` |
| `2` | Version and flags | upper nibble = version, bit `0` = inverted |
| `3..4` | Pitch | signed int16, little-endian centidegrees |
| `5..6` | Roll | signed int16, little-endian centidegrees |
| `7` | Checksum | uint8 sum of bytes `0..6` |

There is no sequence number. Dropped frames are acceptable for this HUD; the
controller simply uses the newest valid frame. The sensor node also throttles
transmit to `IMU_LINK_SEND_INTERVAL_MS`, currently `20 ms`, so the stream is
about 50 frames per second. At 8 bytes per frame this uses about 4000 bits per
second on the UART after start/stop bits are included, leaving useful headroom
for a 9600 baud 433 MHz serial link.

The receiver parser is non-blocking and consumes all available UART bytes each
HUD frame. It uses the magic bytes to resynchronise after dropped or noisy data.
If no valid frame is received for `IMU_LINK_TIMEOUT_MS`, currently `250 ms`, the
display controller shows `NO UART`. The current loop suppresses alert sounds
while waiting for the link. When `SHOW_IMU_LINK_DIAGNOSTICS` is enabled, the
error screen also shows byte, valid frame, checksum error, and version error
counters.

## Controls

### Button A

- Hold then release for at least `100 ms`:
  - In sensor-node mode, resets the reference attitude to the current filtered
    pitch and roll.
  - In sensor-node mode, updates the inversion reference accelerometer vector.
  - In sensor-node mode, plays a `500 Hz` feedback beep for `1000 ms`, if sound
    is enabled.
  - In display-controller mode, this gesture is ignored because the reference is
    determined by the remote sensor node.

- Hold for `1000 ms`:
  - Toggles the plot geometry between circle mode and ellipse mode.
  - This long-hold path is separated from the short reset path so the reference
    is not reset while toggling geometry.

### Button B

- Hold then release for at least `100 ms`:
  - Cycles through the configured level/warning limit states.
  - Plays a `3000 Hz` beep for `100 ms`, if sound is enabled.

- Hold for `1000 ms`:
  - Toggles all normal beeps off or on.
  - Plays a three-tone confirmation sequence even when turning sound off:
    - Sound on: `784 -> 1047 -> 1319`
    - Sound off: `1319 -> 1047 -> 784`
  - Each tone lasts `100 ms`.

### AXP Button

`checkAXPPress()` watches the AXP button and restarts the ESP32 when pressed and
released.

## Limit States

Button B cycles through these states:

| State | Level limit | Warning limit | Plot limit |
| --- | ---: | ---: | ---: |
| 1 | `5.0` deg | `15.0` deg | `35.0` deg |
| 2 | `7.5` deg | `17.5` deg | `35.0` deg |
| 3 | `10.0` deg | `20.0` deg | `35.0` deg |
| 4 | `12.5` deg | `22.5` deg | `35.0` deg |

The alert state is based on the larger absolute value of pitch or roll, unless
the device is inverted. Inversion always forces alarm.

## Plot Geometry

The HUD can draw the lower attitude plot as either a circle or an ellipse.
Startup defaults to ellipse mode.

### Circle Mode

- Centre: `PLOT_CENTER_X`, `PLOT_CENTER_Y`.
- Radius: `PLOT_RADIUS`.
- Current values: centre at approximately `67, 180`, radius `55`.
- The cube reference is only prepared and drawn in circle mode, and only when the
  compile-time `showCube` setting is true.

### Ellipse Mode

- The ellipse starts `10 px` below the top of the display and extends to the
  bottom.
- Centre Y and radius Y are derived from `ELLIPSE_TOP_OFFSET`.
- Roll uses the horizontal radius.
- Pitch uses the taller vertical radius, intentionally exaggerating pitch
  movement because pitch is considered more important to stabilise.

### Shared Plot Styling

- The outer boundary and crosshair axes always use the same colour as the
  current spot state.
- Line width is `3 px`.
- The y-axis has a solid forward arrow at the top:
  - `FORWARD_ARROW_LENGTH`
  - `FORWARD_ARROW_HALF_WIDTH`
  - `FORWARD_ARROW_FLUTE_DEPTH`
- The arrow uses a small fluted cut-out at the base.

## Attitude Labels

Pitch and roll are displayed as separate, zero-decimal labels:

- Pitch: x `20`, y `5`, currently `TFT_GREENYELLOW`.
- Roll: x `115`, y `5`, currently `TFT_MAGENTA`.

These are drawn after the plot and spot so they remain visible over the filled
plot background.

## Spot Behaviour

The spot centre is mapped from pitch and roll into the current plot geometry
using `plot_limit_degrees`. The centre is allowed to reach the outer plot
boundary, meaning part of the spot can be clipped visually by the plot edge.

### Level

- Condition: within `level_limit_degrees` on both axes.
- Spot: large filled green spot.
- Diameter: `40 px` because `LEVEL_SPOT_RADIUS` is `20`.
- Label: the floored level limit is drawn in blue at the centre of the green
  spot. For example, `7.5` displays as `7`.

### Warning

- Condition: above the level limit but not above the warning limit.
- Spot: yellow.
- Diameter: `20 px` because `SPOT_RADIUS` is `10`.

### Alarm

- Condition: above `warning_limit_degrees`, or inverted.
- Spot: large red/orange alarm spot.
- Diameter: `40 px`.
- Flash: currently one orange tick then two red ticks, driven by
  `ALARM_SPOT_FLASH_TICK_MS`.
- When the spot is red and not hollow, the warning limit is drawn in white at
  zero decimal places.

## Inversion Handling

Pitch and roll alone cannot distinguish normal-flat from upside-down-flat, so
the HUD also tracks the raw accelerometer direction.

Implementation outline:

- `readTiltSample()` reads pitch, roll, and an `AccelVector`.
- Startup stores the initial `AccelVector` as the inversion reference.
- Button A reference reset also updates this inversion reference.
- `isInvertedFromReference()` computes a normalized dot product between the
  current and reference acceleration vectors.
- If the dot product is below `INVERSION_DOT_THRESHOLD`, currently `0.0`, the
  device is considered inverted.
- Inversion forces `HeadsUpAlertState::Alarm`.

Inverted alarm spots are hollow:

- The outer alarm spot is drawn in the current alarm colour.
- The centre is filled black using `HOLLOW_SPOT_RING_WIDTH`.
- The warning-limit text is not drawn inside hollow inverted spots.

## Return Arrow

Alarm state can draw a magenta arrow protruding from the large alarm spot.
Dimensions are configurable:

- `RED_SPOT_RETURN_ARROW_LENGTH`
- `RED_SPOT_RETURN_ARROW_HEAD_LENGTH`
- `RED_SPOT_RETURN_ARROW_HEAD_WIDTH`
- `RED_SPOT_RETURN_ARROW_SHAFT_WIDTH`
- `RED_SPOT_RETURN_ARROW_SPOT_OVERLAP`
- `RED_SPOT_RETURN_ARROW_COLOR`

The arrow is drawn before the alarm spot, so the spot covers the tail and the
arrow appears to protrude from it.

Normal alarm behaviour:

- The arrow points from the alarm spot back toward the plot origin.

Inverted alarm behaviour:

- The arrow points away from the origin, indicating the movement direction that
  should flip the unit back over.
- If the hollow inverted spot is exactly at the origin, the arrow falls back to
  pointing upward so there is still a visible cue.

## LED And Buzzer Behaviour

The red LED uses `setRedLed()`, which accounts for the active-low M5StickC Plus
LED.

### Level

- The buzzer is muted unless a feedback tone is active.
- The red LED gives a `5 ms` pulse every `3000 ms` as a heartbeat-style good
  attitude indicator.

### Warning

- The red LED flashes at a `250 ms` cadence.
- No warning buzzer tone is played.

### Alarm

- The red LED is held on.
- The buzzer alternates `2000 Hz` on/off at a `250 ms` cadence, unless:
  - sound is disabled,
  - a feedback beep is active,
  - or the sound toggle flourish is active.

### Sound Toggle

`SoundEnabled` gates normal beeps. The sound toggle flourish itself is allowed
to play so the user gets confirmation when sound is disabled or re-enabled.

## Implementation Map

Important functions and responsibilities:

- `MPU6886Test_heads_up(bool show_cube)`
  - Main HUD loop.
  - In sensor-node mode, reads and filters local IMU data.
  - In display-controller mode, reads remote UART attitude frames.
  - Handles Button A and Button B gestures.
  - Updates pitch/roll deltas and inversion state.
  - Sends remote frames when acting as the sensor node.
  - Clears the sprite, draws cube if enabled, draws the HUD plot, updates alerts,
    then pushes the sprite.

- `sendImuLinkSample()`
  - Packs referenced pitch, referenced roll, and inverted flag into the 8-byte
    UART frame format.
  - Throttles transmission using `IMU_LINK_SEND_INTERVAL_MS`.

- `readRemoteImuSample()`
  - Non-blocking UART receive/parser loop.
  - Keeps the newest valid sample in `RemoteImuSample`.

- `remoteImuSampleIsFresh()`
  - Rejects stale remote samples after `IMU_LINK_TIMEOUT_MS`.

- `readTiltSample()`
  - Reads accelerometer data.
  - Computes pitch/roll angles.
  - Optionally returns the raw acceleration vector for inversion detection.

- `getHeadsUpAlertState()`
  - Converts pitch/roll/inversion into `Level`, `Warning`, or `Alarm`.

- `drawHeadsUpPlot()`
  - Maps pitch/roll into circle or ellipse coordinates.
  - Chooses spot colour and size.
  - Draws plot background, boundary, axes, forward arrow, spot, spot labels, and
    attitude labels.

- `headsUpPlotGeometry()`
  - Returns circle or ellipse dimensions for the active geometry mode.

- `drawRedSpotReturnArrow()`
  - Draws the directional arrow associated with alarm correction.
  - Reverses direction for inverted alarms.

- `updateHeadsUpAlerts()`
  - Drives LED and buzzer state asynchronously.
  - Gives precedence to sound toggle tones and feedback beeps over alarm tones.

- `startFeedbackBeep()`, `updateFeedbackBeep()`
  - Handle one-shot feedback tones for Button A and Button B short actions.

- `toggleSoundEnabled()`, `updateSoundToggleTones()`
  - Handle global sound enable/disable and its three-tone confirmation.

- `prepareHeadsUpCube()`, `drawHeadsUpCube()`
  - Keep the cube reference rendering separate from the HUD plot.
  - Called only when the active geometry is circle mode and `showCube` is true.

## Timing And Responsiveness

The HUD path should avoid blocking calls. Timing is handled with `millis()` and
state variables.

Important conventions:

- Do not add `delay()` calls to `MPU6886Test_heads_up()`.
- Keep UART receive non-blocking. Do not wait for a full packet inside the HUD
  loop.
- Use `pressedFor()` and `releasedFor()` from the M5StickC Plus button class for
  button gestures.
- Long-press actions should be separated from short-press actions so the short
  action is not triggered while reaching the long-press threshold.
- Audio patterns should be sequenced asynchronously, not with blocking sleeps.

## Design Notes

- The overall style is inspired by the classic 8-bit game Elite: simple
  geometry, saturated colours, strong warning states, and minimal text.
- The cube is kept as an optional reference, not as the primary UI.
- The ellipse mode is intentionally biased toward pitch sensitivity by giving
  pitch a taller plotting range.
- The filled plot background uses a 70 percent darker version of the active
  spot colour unless `PLOT_BACKGROUND_BLACK` is set.
- The hollow inverted spot is deliberately different from the normal alarm spot
  because inverted-flat can otherwise look deceptively level.

## Future Feature Ideas

- Add hysteresis between level, warning, and alarm states to reduce boundary
  chatter.
- Persist user settings in non-volatile storage:
  - current limit state,
  - sound enabled/disabled,
  - circle versus ellipse mode,
  - cube visible/hidden.
- Add a small sound-off indicator on screen.
- Add a small geometry-mode indicator on screen.
- Add a battery indicator and brightness control.
- Add a startup splash or self-test screen styled like an old vector display.
- Add a simulator mode that feeds synthetic pitch/roll/inversion data into
  `drawHeadsUpPlot()` for desktop screenshots or unit tests.
- Add a small UART link status indicator showing fresh/stale samples and valid
  frame movement.
- Add a simple role splash screen so it is obvious whether a flashed unit is a
  display controller or sensor node.
- Consider sending raw accelerometer vectors as an optional second frame type if
  future controller-side diagnostics need more than the inverted flag.
- Split the sketch into modules:
  - IMU/reference handling,
  - button gesture handling,
  - audio/LED alert handling,
  - HUD rendering.
- Improve inversion guidance by deriving the best corrective flip axis from the
  raw accelerometer vector, rather than using only the plot-origin direction.
- Add configurable colour palettes, including an even more Elite-like amber or
  green vector display mode.
- Add optional numeric labels for the active level/warning state outside the
  spot for debugging.
- Add serial debug output behind a compile-time flag for IMU vectors, dot
  product, alert state, and button gesture state.
- Add automated build verification in CI for PlatformIO.
