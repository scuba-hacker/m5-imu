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
- `showCube` controls whether the cube reference is drawn. It is currently
  `true`.

## Controls

### Button A

- Hold then release for at least `100 ms`:
  - Resets the reference attitude to the current filtered pitch and roll.
  - Updates the inversion reference accelerometer vector.
  - Plays a `500 Hz` feedback beep for `1000 ms`, if sound is enabled.

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

### Circle Mode

- Centre: `PLOT_CENTER_X`, `PLOT_CENTER_Y`.
- Radius: `PLOT_RADIUS`.
- Current values: centre at approximately `67, 180`, radius `55`.

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
  - Reads and filters IMU data.
  - Handles Button A and Button B gestures.
  - Updates pitch/roll deltas and inversion state.
  - Clears the sprite, draws cube if enabled, draws the HUD plot, updates alerts,
    then pushes the sprite.

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

## Timing And Responsiveness

The HUD path should avoid blocking calls. Timing is handled with `millis()` and
state variables.

Important conventions:

- Do not add `delay()` calls to `MPU6886Test_heads_up()`.
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
