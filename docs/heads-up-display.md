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
- Preserve the rotating cube from the original factory application for the M5
  Stick C Plus as an optional reference view.
- Support a two-device controller/sensor arrangement where the display controller
  receives attitude data from a remote sensor node over UART.
- Make the lower plot feel like an 8-bit attitude gauge: stark geometry,
  high-contrast colour states, and simple directional cues.
- Keep the HUD loop responsive. All user input, LED pulses, buzzer patterns,
  alarm flashing, and sound toggle tones are asynchronous and based on
  `millis()`.
- Treat startup, or the end of a delayed Button A reference reset countdown, as
  the nominal level attitude.

## Hardware And Display Setup

- Target device: M5StickC Plus.
- Display orientation: portrait, `135 x 240`.
- Sprite buffer: `TFT_eSprite Disbuff`, pushed using `Displaybuff()`.
- Screen brightness is set in `setup()` through `applyScreenBrightness(true)`.
  This turns on the LCD backlight rail (`LDO2`) and sets brightness to `100`.

```cpp
applyScreenBrightness(true);
```

- A short press of the power button toggles the
  display between:
  - high brightness: `SCREEN_BRIGHTNESS_HIGH`, currently `100`, with `LDO2`
    enabled.
  - low/off brightness: `SCREEN_BRIGHTNESS_LOW`, currently `1`, then `LDO2`
    disabled. 
- Before the brightness changes, the HUD latches and displays a one-second
  overlay showing the current 10-second rolling battery-current average. The
  average is maintained from timestamped AXP battery-current samples, with no
  artificial sample interval. If the signed average indicates charging, the
  overlay adds a red `CHARGING` label so USB-powered tests are not confused with
  battery-discharge readings.
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
- The message rate is drawn near the role marker using a 3-second rolling mean.
  On the sensor node this reflects transmitted frames; on the display controller
  it reflects valid received frames. `MESSAGE_RATE_AS_PERCENT` switches the
  display from messages per second to percent of the configured expected rate.
  Unlike the role marker, message rate is drawn after the plot and spot so it
  remains readable over the ellipse or circle. The displayed text refreshes every
  `MESSAGE_RATE_DISPLAY_UPDATE_MS`, currently `250 ms`, to reduce visual
  flicker.

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
- Button A short hold/release starts a 20-second reference-reset countdown on
  the sensor node. The actual reference is sampled only when the countdown
  completes.
- Sends referenced pitch, referenced roll, inverted state, and reference-reset
  countdown state over UART.
- Still draws the local HUD, drives its own LED/buzzer, and responds to all
  local buttons.
- Starts with sound disabled by default, so local alarm tones are off unless
  Button B long-hold is used to enable sound.
- Shows `S` as its static role marker.

### Display Controller

Set `DEVICE_IS_SENSOR_NODE` to `false` for the display device.

- Does not read the local IMU in the HUD loop.
- Receives referenced pitch, referenced roll, and inverted state from the sensor
  node.
- Mutes its own alarm sound while fresh sensor frames report that the remote
  reference-reset countdown is active. This lets the sensor be repositioned
  during the countdown without the controller sounding an intentional alarm.
- Shows a `Levelling` banner while fresh sensor frames report that
  the remote reference-reset countdown is active.
- Drives the display, LED, buzzer, alarm logic, limit cycling, sound toggle, and
  circle/ellipse mode locally.
- Starts with sound enabled by default.
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

- Baud rate: controlled by `active_comms_config.imu_link_baud`.
- Radio-link wiring:
  - M5 TX GPIO `32` to HC-12 `RXD`.
  - M5 RX GPIO `33` to HC-12 `TXD`.
  - M5 GPIO `26` to HC-12 `SET`.
- Direct hardwired M5-to-M5 wiring:
  - The role flag swaps RX/TX internally, so the Grove GPIOs can be crossed
    between devices.
- Data format: `SERIAL_8N1`.

When `DEVICE_HAS_RADIO_LINK` is true, both devices use the same local wiring to
their own HC-12 module. The radio link performs the remote crossover. At startup
the firmware releases `SET` for normal mode, briefly pulls it low to enter AT
command mode, scans the supported HC-12 UART baud rates until the module
responds, sends `AT+B...` if the module baud does not match
`active_comms_config.imu_link_baud`, then releases `SET` and opens the IMU data
UART at the configured baud. When the baud is changed, the firmware cycles out
of and back into AT command mode before querying settings with `AT+RX`; this is
needed because the HC-12 may not answer the query immediately after `AT+B...`
without a command-mode cycle.

After a successful HC-12 AT/query exchange, the firmware stores the last
confirmed radio settings in ESP32 Preferences under the `hc12` namespace:

- `baud`: last confirmed HC-12 UART baud.
- `mode`: last confirmed/assumed HC-12 FU mode. This is currently `FU3` because
  mode-changing is not implemented yet, but the storage is ready for it.
- `valid`: whether the stored values are meaningful.

If the `SET` pin is not wired or AT mode cannot be reached, the firmware cannot
change or query the module. In that case it compares the compiled settings with
the last confirmed values from flash. If the values differ, the HUD diverts to a
UART mismatch warning screen in either sensor-node or display-controller mode.
The title is shown as two lines, `UART` and `MISMATCH`, and the screen shows
`LAST <baud> FUx` and `CFG <baud> FUx` so the user can spot a likely radio
configuration mismatch.

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
| `2` | Version and flags | upper nibble = version, bit `0` = inverted, bit `1` = reference-reset countdown active |
| `3..4` | Pitch | signed int16, little-endian centidegrees |
| `5..6` | Roll | signed int16, little-endian centidegrees |
| `7` | Checksum | uint8 sum of bytes `0..6` |

There is no sequence number. Dropped frames are acceptable for this HUD; the
controller simply uses the newest valid frame. The sensor node also throttles
transmit using `active_comms_config.imu_link_send_interval_ms`. The throttle is
anchored to a fixed next-send schedule, so a late display frame does not shift
all future sends later. Before writing, the sender checks that the UART TX
buffer has room for a complete frame; if not, it skips that stale sample rather
than blocking the display loop. For example, a `20 ms` interval gives about
50 frames per second. At 8 bytes per frame this uses about 4000 bits per second
on the UART after start/stop bits are included, leaving useful headroom for a
9600 baud 433 MHz serial link.

The receiver parser is non-blocking and consumes all available UART bytes each
HUD frame. It uses the magic bytes to resynchronise after dropped or noisy data.
If no valid frame is received for
`active_comms_config.imu_link_timeout_ms`, the display controller shows
`NO UART`, or `UART` / `MISMATCH` when stored radio settings disagree with the
compiled settings. The current loop suppresses alert sounds while waiting for
the link. When `SHOW_IMU_LINK_DIAGNOSTICS` is enabled, the error screen also
shows byte, valid frame, checksum error, version error, and remote
reset-countdown state.

The frame size and protocol version did not change when the countdown flag was
added; it reuses spare capacity in the existing flags byte. Both devices still
need firmware containing the flag support if the controller is expected to mute
while the sensor countdown is active.

## Controls

### Button A

- Hold then release for at least `100 ms`:
  - In sensor-node mode, starts a 20-second reference-reset countdown.
  - During the countdown, the sensor shows a large `RESET IN` overlay using the
    bundled `Orbitron_Light_32` font for the number.
  - During the countdown, local sensor sounds are muted. The sensor also sends a
    UART flag so the controller can mute its alarm sound while fresh countdown
    frames are arriving and show a `Levelling` banner.
  - When the countdown completes, the reference attitude is sampled from the
    current filtered IMU state. This gives the user time to press the button and
    then position the sensor on the back of a cylinder before the measurement is
    made.
  - After the first `1000 ms` of the countdown, pressing Button A again requests
    an early finish. The sensor waits until Button A is released, then waits a
    further `1000 ms` before sampling the current filtered IMU state.
  - At countdown completion, the inversion reference accelerometer vector is
    updated and a `500 Hz` feedback beep plays for `1000 ms`, if sound is
    enabled.
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

`checkAXPPress()` watches the AXP/power button. A short press toggles screen
brightness instead of rebooting:

- Before the brightness change, the current 10-second rolling battery-current
  average is shown in a one-second overlay.
- Toggling to bright enables `LDO2` and sets brightness to `100`.
- Toggling to low/off sets brightness to `0` and disables `LDO2`. This LDO2-off
  behavior is currently for current-consumption comparison.

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
- Button A reference reset updates this inversion reference when the 20-second
  countdown completes.
- `isInvertedFromReference()` computes a normalized dot product between the
  current and reference acceleration vectors.
- If the dot product is below `INVERSION_DOT_THRESHOLD`, currently `0.0`, the
  device is considered inverted.
- Inversion forces `HeadsUpAlertState::Alarm`.

Inverted alarm spots are hollow:

- The outer alarm spot is drawn in the current alarm colour.
- The centre is filled black using `HOLLOW_SPOT_RING_WIDTH`.
- The warning-limit text is not drawn inside hollow inverted spots.

## Relative Level Math

The legacy level calculation converts accelerometer readings into two
Euler-style angles and subtracts the saved reference angles:

```cpp
roll_delta = theta - theta_reference;
pitch_delta = phi - phi_reference;
```

That path remains available as the fallback.

The feature flag below enables the newer vector-relative calculation:

```cpp
static constexpr bool USE_VECTOR_RELATIVE_LEVEL = false;
```

When enabled, `calculateVectorRelativeLevelDelta()` compares the current
filtered acceleration vector directly with the saved reference acceleration
vector. This avoids the `atan(accY / accZ)` singularity that can occur when the
sensor is referenced at or near a 90-degree side orientation, where `accZ` can be
close to zero.

The vector-relative path uses the same low-pass smoothing input as the legacy
path. `MPU6886Test_heads_up()` maintains a filtered `AccelVector` alongside the
filtered `theta` and `phi` values, using the same `alpha = 0.2`. Reference resets
store that filtered vector when the countdown completes.

If vector normalization fails or the reference tangent basis cannot be built,
the firmware falls back to the legacy angle subtraction for that frame.

## Return Arrow

Alarm state draws a magenta arrow protruding from the large alarm spot.
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
  - the sound toggle flourish is active,
  - the local sensor reference-reset countdown is active,
  - or the controller is receiving fresh frames that report the sensor
    reference-reset countdown is active.

### Sound Toggle

`SoundEnabled` gates normal beeps. The sound toggle flourish itself is allowed
to play so the user gets confirmation when sound is disabled or re-enabled.
The startup default is role-based: disabled on the sensor node and enabled on
the display controller.

Reference-reset countdown muting does not change `SoundEnabled`. It is a
temporary suppression state used so the sensor can be repositioned while it is
intentionally far from the current reference.

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
  - Packs referenced pitch, referenced roll, inverted flag, and
    reference-reset countdown flag into the 8-byte UART frame format.
  - Throttles transmission using a fixed schedule based on
    `active_comms_config.imu_link_send_interval_ms`.
  - Skips a frame instead of blocking if the UART TX buffer cannot accept a full
    8-byte frame immediately.

- `setupImuLinkSerial()`, `setHc12BaudRate()`
  - Release/pull the HC-12 `SET` pin through `HC12_SET_GPIO`.
  - In radio-link mode, enter HC-12 AT command mode during startup and set the
    module UART baud to match `active_comms_config.imu_link_baud`.
  - Query the module with `AT+RX` and expect the returned baud, for example
    `OK+B115200`, before entering normal data mode.
  - Store the last confirmed baud and FU mode in ESP32 Preferences.
  - Re-open `ImuLinkSerial` at the final data baud before the HUD loop starts.

- `loadHc12StoredSettings()`, `saveHc12StoredSettings()`
  - Read/write the `hc12` Preferences namespace in flash.
  - Detect when the compiled baud or FU mode differs from the last confirmed
    radio settings.
  - Feed the UART mismatch warning screen when AT mode could not correct/query
    the module.

- `readRemoteImuSample()`
  - Non-blocking UART receive/parser loop.
  - Keeps the newest valid sample in `RemoteImuSample`.
  - Decodes the remote reference-reset countdown flag so the controller can mute
    its own alarm during the sensor countdown.

- `remoteImuSampleIsFresh()`
  - Rejects stale remote samples after
    `active_comms_config.imu_link_timeout_ms`.

- `recordMessageRateEvent()`, `averageMessageRatePerSecond()`
  - Track sent or valid received frames in buckets over the last 3 seconds.
  - Use a rolling mean because packet-loss testing needs the average delivered
    frame rate rather than an outlier-resistant median.

- `readTiltSample()`
  - Reads accelerometer data.
  - Computes pitch/roll angles.
  - Optionally returns the raw acceleration vector for inversion detection.

- `calculateVectorRelativeLevelDelta()`
  - Optional feature-flagged replacement for subtracting reference Euler angles.
  - Compares the current filtered acceleration vector against the saved filtered
    reference vector.
  - Falls back to the legacy angle path if it cannot produce a stable basis.

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
  - Suppresses buzzer output while a local or remote reference-reset countdown
    is active.

- `startReferenceResetCountdown()`, `drawReferenceResetCountdownOverlay()`,
  `drawRemoteLevelingBanner()`, `referenceResetCountdownComplete()`
  - Manage the 20-second delayed sensor reference reset.
  - Allow Button A to request early countdown completion after the first
    `1000 ms`, then wait for release plus a `1000 ms` settle period.
  - Draw the large sensor countdown overlay and the controller `Levelling`
    banner for fresh remote countdown frames.
  - Temporarily mute sound during the countdown.

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
- Reference-reset countdown timing must remain non-blocking; the HUD keeps
  drawing, sending/receiving UART frames, and updating LEDs while waiting for
  the countdown to complete.

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

## Test Log

### HC-12 115200 Baud Test, 2026-04-28

Goal: configure both M5StickC Plus UARTs and both HC-12 modules to `115200`
baud, then verify the sensor-to-controller attitude stream.

Procedure:

- Sensor stick on `/dev/cu.usbserial-5D5222C816` was flashed with
  `DEVICE_IS_SENSOR_NODE = true` and
  `active_comms_config = comms_unthrottled_115200_air`.
- The sensor entered HC-12 AT mode through GPIO `26`, found the module still at
  `9600`, sent `AT+B115200`, then confirmed `115200` with `AT+RX` on the next
  startup.
- The baud-change path was improved to cycle out of and back into HC-12 AT mode
  after a real `AT+B...` change before running `AT+RX`.
- Controller stick on `/dev/cu.usbserial-D55208AFE7` was flashed with
  `DEVICE_IS_SENSOR_NODE = false` and
  `active_comms_config = comms_unthrottled_115200_air`.

Observed controller AT diagnostics:

```text
HC12 TX: "AT+B115200", expecting "OK+B115200"
HC12 RX: "OK+B115200"
HC12 TX: "AT+RX", expecting "OK+B115200"
HC12 RX: "OK+B115200
OK+RC001"
HC12: normal data mode at 115200 baud
```

Observed controller receive diagnostics:

```text
IMU RX: bytes=4288 frames=535 rate=37.7/s checksum=0 version=0 fresh=yes
IMU RX: bytes=6432 frames=803 rate=37.7/s checksum=0 version=0 fresh=yes
IMU RX: bytes=7656 frames=956 rate=37.3/s checksum=0 version=0 fresh=yes
```

Conclusion: `115200` works cleanly with the current two-HC-12 bench setup. The
message rate remains about `37/s`, which matches the existing display-loop limit
rather than a UART/radio limit. During the captured controller run there were no
checksum errors, no version errors, and the link stayed fresh.

### HC-12 1200 Baud Unthrottled Test, 2026-04-28

Goal: configure both M5StickC Plus UARTs and both HC-12 modules to `1200` baud,
run the sensor unthrottled, and observe the practical maximum delivered frame
rate.

Procedure:

- Sensor stick on `/dev/cu.usbserial-5D5222C816` was flashed with
  `DEVICE_IS_SENSOR_NODE = true` and
  `active_comms_config = comms_unthrottled_1200_water`.
- The sensor found its HC-12 command interface at `115200`, sent `AT+B1200`,
  cycled command mode, and confirmed `OK+B1200` with `AT+RX`.
- Controller stick on `/dev/cu.usbserial-D55208AFE7` was flashed with
  `DEVICE_IS_SENSOR_NODE = false` and
  `active_comms_config = comms_unthrottled_1200_water`.
- The controller also changed its HC-12 from `115200` to `1200` and confirmed
  `OK+B1200`.

Observed controller receive diagnostics:

```text
IMU RX: bytes=1214 frames=148 rate=15.0/s checksum=0 version=1 fresh=yes
IMU RX: bytes=2067 frames=255 rate=15.0/s checksum=0 version=1 fresh=yes
IMU RX: bytes=3041 frames=376 rate=15.0/s checksum=0 version=1 fresh=yes
```

Conclusion: unthrottled `1200` reaches about `14.7-15.0` valid frames per
second, which is the expected UART ceiling for 8-byte frames on 8N1 serial:
`1200 / (8 bytes * 10 bits) = 15 frames/s`. The non-blocking
`availableForWrite()` guard prevents display stalls while stale samples are
dropped before entering the UART queue. The captured run stayed fresh, showed
no checksum errors, and had one version-counter increment during startup or
resynchronisation.

### HC-12 1200 Baud 12 Hz Test, 2026-04-28

Goal: keep the HC-12 modules at `1200` baud but throttle the sensor to the
`83 ms` fixed schedule, giving about `12 Hz` and leaving radio/UART headroom.

Procedure:

- Sensor stick on `/dev/cu.usbserial-5D5222C816` was flashed with
  `DEVICE_IS_SENSOR_NODE = true` and
  `active_comms_config = comms_12_Hz_1200_water`.
- The sensor confirmed the HC-12 was already at `OK+B1200`.
- Controller stick on `/dev/cu.usbserial-D55208AFE7` was flashed with
  `DEVICE_IS_SENSOR_NODE = false` and
  `active_comms_config = comms_12_Hz_1200_water`.
- The controller confirmed the HC-12 was already at `OK+B1200`.

Observed controller receive diagnostics:

```text
IMU RX: bytes=1172 frames=143 rate=12.3/s checksum=0 version=1 fresh=yes
IMU RX: bytes=1854 frames=228 rate=12.0/s checksum=0 version=1 fresh=yes
IMU RX: bytes=2539 frames=313 rate=11.7/s checksum=0 version=1 fresh=yes
```

Conclusion: the `12 Hz` schedule works cleanly at `1200` baud. It delivers
about `11.7-12.3` valid frames per second, keeps the link fresh, and leaves
roughly 20 percent serial headroom compared with the 15 frame/s line-rate
ceiling. The captured run showed no checksum errors and one version-counter
increment during startup or resynchronisation. This is a better candidate than
unthrottled `1200` for underwater/range testing because it avoids running the
link continuously at saturation.

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
- Add a persistent battery indicator and brightness setting.
- Add a startup splash or self-test screen styled like an old vector display.
- Add a simulator mode that feeds synthetic pitch/roll/inversion data into
  `drawHeadsUpPlot()` for desktop screenshots or unit tests.
- Add a small UART link status indicator showing fresh/stale samples and valid
  frame movement.
- Add a simple role splash screen so it is obvious whether a flashed unit is a
  display controller or sensor node.
- Consider sending raw accelerometer vectors as an optional second frame type if
  future controller-side diagnostics need more than the current status flags.
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
