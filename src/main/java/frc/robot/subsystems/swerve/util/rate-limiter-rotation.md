# Rotation in `SwerveRateLimiter`

Why the rate limiter needs to know about rotation, what the math is, and what was actually
implemented. Numbers are for the 2026 drivetrain: `wheelBase` 0.55372 m, `trackWidth`
0.55118 m, `maxSpeed` 7.0 m/s.

## The problem

Team 1690's scheme, as presented, limits `wantedAcc` — a translational acceleration
derived from a chassis velocity delta. The original implementation followed that
faithfully and therefore ignored `omega` entirely: angular acceleration was unbounded, and
the traction and skid limits were computed from `hypot(vx, vy)` and `hypot(ax, ay)` as
though the robot were never rotating.

That is a real gap, not a cosmetic one. A swerve module does not experience chassis
acceleration; it experiences its own. For a module at offset `r_i` from the chassis centre,
in the robot frame:

```text
v_i = v + omega x r_i
a_i = a + alpha x r_i - omega^2 * r_i
```

Written out in components, with `r_i = (x_i, y_i)`:

```text
v_i = (vx - omega*y_i,  vy + omega*x_i)
a_i = (ax - alpha*y_i - omega^2*x_i,  ay + alpha*x_i - omega^2*y_i)
```

The worst-case lever arm on this robot is

```text
r = hypot(wheelBase/2, trackWidth/2) = hypot(0.27686, 0.27559) = 0.39064 m
```

Three consequences follow, and the limiter now accounts for all three.

## 1. The centripetal term consumes the friction budget

`omega^2 * r_i` is always present when rotating, points inward, and does not depend on any
commanded acceleration. It is spent before the driver asks for anything:

| omega (rad/s) | `omega^2 * r` (m/s²) | share of a 10 m/s² budget |
| ------------- | -------------------- | ------------------------- |
| 2             | 1.56                 | 16%                       |
| 4             | 6.25                  | 63%                       |
| 5.06          | 10.0                 | 100%                      |
| 6             | 14.06                | 141% — over budget        |

Teleop clamps `omega` to `Constants.Swerve.maxAngularVelocity = 4.0`, so a full-speed spin
already consumes ~63% of the tires' capacity before any translation is commanded.
`TurnToRotation` uses `Constants.SwerveTransformPID.maxAngularVelocity = 6.0`, which on its
own exceeds what the carpet can hold. A skid limit that caps only `hypot(ax, ay)` cannot be
correct under those conditions — it will happily permit the full translational budget on
top of a spin that has already spent it.

This also yields a derived ceiling worth carrying into tuning:

```text
omega_max = sqrt(skidAccelLimit / r) ~= 5.06 rad/s   (at skidAccelLimit = 10)
alpha_max = skidAccelLimit / r       ~= 25.6 rad/s^2
```

Both existing `maxAngularVelocity` constants and
`Constants.SwerveTransformPID.maxAngularAcceleration = 45` should be re-examined against
those figures once `skidAccelLimit` is measured. 45 rad/s² is roughly 1.8x the value the
tires can deliver even with zero translation commanded.

## 2. The traction falloff belongs on the fastest module, not the chassis

The falloff `maxAcc * (1 - v/vMax)` models motor torque dropping off with rotor speed. That
is a per-module effect. When translating and rotating simultaneously the outer module runs
much faster than the chassis centre — at `omega` = 8 rad/s the modules are doing 3.1 m/s
while `hypot(vx, vy)` reads zero — so keying the falloff on chassis speed overstates the
available acceleration and asks the outer module for torque it cannot produce.

Fix: `maxModuleSpeed = max_i |v + omega x r_i|`, four `hypot` calls. This is what
`tractionFalloffUsesFastestModuleNotChassisCentre` in the test pins down.

## 3. Tip is genuinely translation-only

Rotation about the vertical axis produces no horizontal CG acceleration when the CG sits at
the rotation centre, so the tip clamps stay translational. With a CG offset `d` the
rotation contributes `alpha x d - omega^2 * d`; for realistic offsets (a few cm) that is
small next to the translational terms. The assumption is documented in the class Javadoc
rather than modelled — if the 2027 robot carries a badly offset CG, this is the term to
revisit.

## What was implemented

Two additions to the pipeline.

**An angular acceleration clamp**, symmetric, applied before the shared budget below:

```text
alpha = clamp(alpha, -angularAccelLimit, +angularAccelLimit)
```

**A shared skid budget**, using the triangle inequality with the centripetal term reserved
out first:

```text
avail  = max(0, skidAccelLimit - omega^2 * r)
demand = hypot(ax, ay) + |alpha| * r
if (demand > avail) { s = avail / demand;  ax *= s;  ay *= s;  alpha *= s; }
```

Because `|a + alpha x r_i| <= |a| + |alpha| * r`, bounding the sum bounds every module. The
result is conservative — it assumes the translational and tangential terms are aligned,
which for any single module they generally are not — but it is never optimistic, it is
closed-form, and scaling all three components together preserves the driver's requested
translation-to-rotation ratio instead of sacrificing one for the other.

Note what the centripetal reservation buys: at `omega` large enough that
`omega^2 * r > skidAccelLimit`, `avail` is zero and *no* change in velocity is permitted.
The robot is already asking more of the tires than they have, so the correct answer is to
add nothing. `skidSuppressesTranslationWhenCentripetalExceedsBudget` covers that case.

## What was not implemented

**The exact per-module friction circle.** Instead of the triangle inequality, solve for the
largest scale `s` in `[0, 1]` satisfying, for every module,

```text
| s * (a + alpha x r_i) - omega^2 * r_i |  <=  skidAccelLimit
```

Expanding gives a quadratic in `s` per module:

```text
|u_i|^2 s^2  -  2 (u_i . c_i) s  +  |c_i|^2 - skidAccelLimit^2  <=  0
      where  u_i = a + alpha x r_i,   c_i = omega^2 * r_i
```

Take the smaller positive root of each and the minimum over the four modules. This is a
drop-in replacement for the skid block — same inputs, same output shape, ~20 lines and no
allocation. It recovers the agility the triangle inequality gives away, and it correctly
recognises that the centripetal term can *oppose* the commanded acceleration on some
modules rather than always adding to it. Worth doing if the conservatism proves to cost
noticeable agility once `skidAccelLimit` is actually measured; not worth doing before then,
because with `skidAccelLimit` at `POSITIVE_INFINITY` neither formulation does anything.

**Rotation in the tip clamps**, as above.

**A rotational analogue of the traction falloff.** The falloff currently reduces the
translational cap based on module speed but does not cap `alpha` by the same reasoning —
a module already near free speed cannot contribute much tangential force either. In
practice `angularAccelLimit` and the shared skid budget cover this; separating it out only
matters if angular authority at high speed turns out to be mistuned.
