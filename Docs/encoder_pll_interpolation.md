# Encoder PLL, Interpolation, and Electrical Phase

This note explains the three important parts in `MotorControl/encode.c`:

- `encoder_run_pll()`
- `encoder_run_interpolation()`
- `encoder_publish_phase()`

The short version:

```text
encoder raw count
    -> PLL estimates smooth position and velocity
    -> interpolation estimates position between two counts
    -> electrical phase is calculated for FOC
```

## 1. Raw Encoder Count

The encoder gives a position in counts.

For incremental ABZ:

```text
TIM3->CNT
```

For AS5047P SPI:

```text
pos_abs = raw & 0x3fff
```

If `cpr = 16384`, one mechanical turn is split into 16384 small grids.

Example:

```text
count = 0       means 0 turn
count = 8192    means 0.5 turn
count = 16384   means 1 turn
```

But the encoder count is still a stair-step signal. It jumps from one integer
count to the next integer count. It does not directly tell us the smooth
position between two counts.

That is why YDrive uses PLL and interpolation.

## 2. Why Use PLL If We Already Have Real Position

The encoder gives real position, but only at discrete sample times and discrete
count steps.

FOC wants smoother information:

```text
position
velocity
electrical phase
electrical velocity
```

The encoder directly gives position, but velocity is not directly measured.
Velocity must be estimated from position changes.

A simple velocity formula is:

```text
velocity = (new_count - old_count) / Ts
```

This works, but it is noisy. If the count sometimes jumps by 0, sometimes by 1,
the calculated velocity will shake.

The PLL is like a smooth follower:

```text
I guess where the encoder should be.
Then I compare my guess with the real encoder.
If I guessed behind, I speed up.
If I guessed ahead, I slow down.
```

So the PLL has two jobs:

```text
1. smooth position
2. estimate velocity
```

## 3. PLL Code Flow

In `encoder_run_pll()`:

```c
encoder_state.pos_estimate_counts +=
	current_meas_period * encoder_state.vel_estimate_counts;

encoder_state.pos_cpr_counts +=
	current_meas_period * encoder_state.vel_estimate_counts;
```

Formula:

```text
x_guess = x_guess + Ts * v_guess
```

Meaning:

```text
Use the estimated velocity to predict where the rotor should be now.
```

Then YDrive compares the real encoder position with the predicted position:

```c
float delta_pos_counts =
	(float)(encoder_state.shadow_count -
	        (int32_t)encoder_state.pos_estimate_counts);

float delta_pos_cpr_counts =
	(float)(encoder_state.count_in_cpr -
	        (int32_t)encoder_state.pos_cpr_counts);

delta_pos_cpr_counts =
	wrap_pm(delta_pos_cpr_counts, (float)encoder_config.cpr);
```

Formula:

```text
e = x_encoder - x_guess
```

Meaning:

```text
e is the position error.
```

Then PLL corrects position:

```c
encoder_state.pos_estimate_counts +=
	current_meas_period * encoder_state.pll_kp * delta_pos_counts;

encoder_state.pos_cpr_counts +=
	current_meas_period * encoder_state.pll_kp * delta_pos_cpr_counts;
```

Formula:

```text
x_guess = x_guess + Ts * kp * e
```

Meaning:

```text
If the guessed position is wrong, pull it toward the real encoder position.
```

Then PLL corrects velocity:

```c
encoder_state.vel_estimate_counts +=
	current_meas_period * encoder_state.pll_ki * delta_pos_cpr_counts;
```

Formula:

```text
v_guess = v_guess + Ts * ki * e
```

Meaning:

```text
If the position error stays positive, the guessed velocity is too slow.
If the position error stays negative, the guessed velocity is too fast.
```

This is why the PLL can estimate velocity.

## 4. PLL Gains

YDrive uses:

```c
pll_kp = 2.0f * bandwidth;
pll_ki = 0.25f * pll_kp * pll_kp;
```

Formula:

```text
kp = 2 * bandwidth
ki = 0.25 * kp^2
```

Small-white-box meaning:

```text
bandwidth decides how fast the PLL follows the encoder.
```

If bandwidth is larger:

```text
PLL follows faster
velocity responds faster
noise is easier to follow
```

If bandwidth is smaller:

```text
PLL is smoother
velocity is quieter
response is slower
```

YDrive currently uses:

```text
ENCODER_BANDWIDTH_DEFAULT = 1000.0
```

That is a common practical starting value.

## 5. Why Interpolation Is Still Needed

PLL estimates smooth position and velocity, but FOC finally needs electrical
phase at every control step.

The raw encoder count is still integer:

```text
100
101
102
```

But the rotor moves continuously:

```text
100.1
100.2
100.3
...
100.9
101.0
```

Interpolation estimates where the rotor is inside the current encoder count.

In code:

```c
if (snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
{
	encoder_state.interpolation = 0.5f;
}
else if (delta_enc > 0)
{
	encoder_state.interpolation = 0.0f;
}
else if (delta_enc < 0)
{
	encoder_state.interpolation = 1.0f;
}
else
{
	encoder_state.interpolation +=
		current_meas_period * encoder_state.vel_estimate_counts;

	encoder_state.interpolation =
		clampf(encoder_state.interpolation, 0.0f, 1.0f);
}
```

The interpolation value means:

```text
0.0  just entered this count
0.5  middle of this count
1.0  almost entering the next count
```

If a new encoder edge arrives:

```text
delta_enc > 0  -> interpolation = 0.0
delta_enc < 0  -> interpolation = 1.0
```

If no new count arrives:

```text
interpolation = interpolation + Ts * velocity_counts
```

Meaning:

```text
Even if the integer count did not change this time,
the rotor may still be moving inside this small grid.
Use PLL velocity to move the in-grid progress forward.
```

Then:

```c
return (float)corrected_enc + encoder_state.interpolation;
```

Formula:

```text
interpolated_enc = corrected_count + interpolation
```

Example:

```text
corrected_count = 100
interpolation = 0.35
interpolated_enc = 100.35
```

This makes the electrical phase smoother than using integer counts only.

## 6. How Offset Calibration Gives a Known Open-Loop Electrical Phase

Before offset calibration, YDrive does not yet know this relationship:

```text
encoder count <-> motor electrical phase
```

So it cannot use encoder feedback to control phase. Instead, it uses open-loop
phase.

Open-loop means:

```text
Do not ask the encoder where the rotor is.
Directly command where the stator magnetic field should be.
```

In code:

```c
openloop_controller_.phase_ =
    wrap_pm_pi(0.0f - encoder_config.calib_scan_distance * 0.5f);
```

This directly writes the open-loop electrical phase.

Meaning:

```text
phase_ = -0.5 * calib_scan_distance
```

With the current default:

```text
calib_scan_distance = 16*pi
```

So:

```text
phase_ = -8*pi
```

Electrical phase is periodic:

```text
2*pi = one electrical turn
```

So `-8*pi` is the same direction as `0`.

After:

```c
wrap_pm_pi(-8*pi)
```

the result is:

```text
phase_ = 0
```

So with the current default, this line is effectively:

```c
openloop_controller_.phase_ = 0.0f;
```

The important idea is not the number itself. The important idea is:

```text
Assigning openloop_controller_.phase_ directly tells FOC
where to place the stator magnetic field.
```

If:

```c
openloop_controller_.phase_ = 0.0f;
```

then:

```text
Place the stator magnetic field at electrical phase 0.
```

If:

```c
openloop_controller_.phase_ = M_PI / 2.0f;
```

then:

```text
Place the stator magnetic field at electrical phase 90 deg.
```

This is how the controller "gives" a known electrical phase.

It is not measured. It is commanded.

### Why Use `-scan_distance / 2`

Offset calibration wants a symmetric scan.

The idea is:

```text
start at -scan_distance/2
scan forward by scan_distance
end at +scan_distance/2
```

So the scan is centered around electrical phase 0.

In simple words:

```text
If total scan distance is 100,
start at -50,
move forward 100,
end at +50.
```

For angles:

```text
start_phase = 0 - calib_scan_distance / 2
```

That is exactly this line:

```c
openloop_controller_.phase_ =
    wrap_pm_pi(0.0f - encoder_config.calib_scan_distance * 0.5f);
```

### Lock and Scan

At first:

```c
openloop_controller_.target_vel_ = 0.0f;
```

Meaning:

```text
The commanded magnetic field does not rotate.
The rotor magnet is pulled toward this fixed field direction.
```

This is the lock stage.

Then YDrive sets:

```c
openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;
```

The open-loop controller updates phase like this:

```text
phase = phase + phase_vel * dt
```

Meaning:

```text
The commanded stator magnetic field slowly rotates.
The rotor is dragged by this rotating field.
```

During this scan, YDrive reads encoder counts.

Since YDrive knows:

```text
I commanded this much electrical phase movement.
```

and reads:

```text
The encoder moved this many counts.
```

it can find:

```text
phase_offset
```

That offset is the missing relationship between encoder zero and motor
electrical zero.

## 7. Electrical Phase Output

FOC does not directly use mechanical encoder counts. FOC needs rotor electrical
phase.

One mechanical turn contains `pole_pairs` electrical turns.

For a 7-pole-pair motor:

```text
1 mechanical turn = 7 electrical turns
```

YDrive calculates:

```c
float elec_rad_per_enc =
	(float)motor_config.pole_pairs * 2.0f * M_PI /
	(float)encoder_config.cpr;
```

Formula:

```text
elec_rad_per_count = pole_pairs * 2*pi / cpr
```

Meaning:

```text
Each encoder count corresponds to this many electrical radians.
```

Then:

```c
float phase = elec_rad_per_enc *
              (interpolated_enc - encoder_config.phase_offset_float);
```

Formula:

```text
phase = elec_rad_per_count * (interpolated_count - phase_offset)
```

Meaning:

```text
Convert encoder position into motor electrical angle,
and subtract the offset found by encoder offset calibration.
```

Finally:

```c
encoder_config.phase_ =
	wrap_pm_pi(phase) * (float)encoder_config.direction;
```

Formula:

```text
phase = wrap_pm_pi(phase) * direction
```

Meaning:

```text
Limit phase to -pi..pi and apply encoder direction.
```

Electrical velocity:

```c
encoder_config.phase_vel_ =
	2.0f * M_PI *
	encoder_state.vel_estimate *
	(float)motor_config.pole_pairs *
	(float)encoder_config.direction;
```

Formula:

```text
electrical_velocity =
	2*pi * mechanical_turns_per_second * pole_pairs * direction
```

## 8. Whole Flow In One Picture

```text
ABZ TIM3 or AS5047P SPI
        |
        v
raw encoder count
        |
        v
delta_enc
        |
        v
shadow_count and count_in_cpr
        |
        v
PLL
position estimate + velocity estimate
        |
        v
interpolation
count + in-count fraction
        |
        v
electrical phase
phase_ and phase_vel_
        |
        v
FOC
```

## 9. Practical Debug Points

For AS5047P SPI, send `E` over USB and watch:

```text
abs
raw
spi_flags
spi_err_pm
```

Good SPI result:

```text
spi_flags=0
spi_err_pm=0
abs changes when shaft rotates
```

After offset calibration succeeds:

```text
ready=1
error=0
```

Only when `ready=1`, YDrive publishes valid FOC phase:

```c
if (encoder_state.is_ready)
{
	encoder_config.phase_ = ...
	encoder_config.phase_vel_ = ...
}
```
