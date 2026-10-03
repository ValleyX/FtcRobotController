# QualBotCommand

The Qual Bot rewritten on [valleyLib](https://github.com/ValleyX/valleyLib) command-based,
with autonomous on Pedro Pathing instead of Road Runner.

Currently on **valleyLib 2.0.0** and **Pedro Pathing 3.0.1**. There is no Pedro 3.1 —
upstream's newest tag is `v3.0.1`, and 3.0.1 is what valleyLib 2.0.0 pins.

This sits alongside the original `QualBot` package rather than replacing it, so the
old LinearOpModes still run while this is being validated. Every new opmode name is
prefixed `CB` so the two sets never collide on the Driver Station.

## Pedro 3 migration notes

Pedro 3 is a rewrite, not an upgrade. What changed and where it landed:

| Pedro 2 | Pedro 3 |
| --- | --- |
| single `com.pedropathing:ftc` artifact | `core` + `revhub` |
| `com.pedropathing.geometry.Pose`, `getX()` | `com.pedropathing.math.Pose`, `x()` |
| `Pose` carried a `CoordinateSystem` | bare `(x, y, heading)`, frame is whatever you `setPose` |
| `FollowerBuilder` | `new Follower(localizer, drivetrain, algorithm)` |
| `FollowerConstants` + `PathConstraints` | `ForesightConfig` |
| `MecanumConstants`, `ThreeWheelIMUConstants` | `MecanumConfig`, `ThreeWheelIMUConfig` (lambda over `ConfigVar`) |
| `leftFront` / `leftRear` names | `frontLeft` / `backLeft` |
| `PathChain` + `follower.pathBuilder()` | `Path` + standalone `Paths.line/curve/through` |
| heading set on the builder | set on the path: `.linear()`, `.constant()`, `.tangent()` |
| `setTeleOpDrive(..., isRobotCentric)` | `manual(DrivePowers)` + `ManualDrive.fieldCentric(...)` |
| `turnTo(rad)` | `hold(new Pose(x, y, rad))` |
| `breakFollowing()` | `stop()` — latches IDLE and keeps motors cut |
| `setMaxPower()` | `ForesightConfig.maxPathSpeed` ConfigVar |
| `isLocalizationNAN()`, `isRobotStuck()` | gone; check the pose yourself |

Two things got *better*, not just different:

- **No coordinate conversion.** Because a Pedro 3 `Pose` has no frame attached, every
  waypoint is now the literal FTC-standard centre-origin number the Road Runner autos
  used. The old `ftcPose()` helper is gone.
- **Alliance mirroring is Pedro's job.** `PedroConstants.RED` is a `PoseFactory` carrying
  `mirrorY(0)`, so the near-goal geometry is written once in blue coordinates. Use
  `PedroConstants.FIELD` for poses already written per-alliance — the far-goal autos are,
  so mirroring them would flip numbers that are already right.

valleyLib's own command layer (`CommandOpMode`, `Commands`, `Subsystem`, triggers,
gamepads, decorators) is API-identical between 1.0.8 and 2.0.0, so nothing outside the
Pedro-facing code needed touching.

## Layout

```
QualBotCommand/
  RobotConstants.java        every tuned number, in one place
  PedroConstants.java        follower construction + the FTC-coordinate helper
  QualBotRobot.java          builds all five subsystems

  subsystems/
    DriveSubsystem           Pedro follower: paths in auto, field-centric teleop, heading source
    ShooterSubsystem         flywheel, hood, feed gate
    IntakeSubsystem          roller, beam breaks, ball stop
    VisionSubsystem          Limelight, one frame cached per loop
    LightSubsystem           goBILDA indicator, match countdown

  commands/
    drive/    AimAssistDriveCommand, TurnToHeadingCommand, AimAtGoalCommand
    intake/   IntakeCommand, ExtakeCommand, CollectCommand
    shooter/  ShootCommand, IdleShooterCommand, WarmUpShooterCommand

  teleop/   TeleOpContainer + the blue and red opmodes
  autos/    AutoPaths, CloseAutoGeometry, CloseAutoRoutine, FarAutoRoutines, five opmodes
  tuning/   shooter PIDF, flywheel feedforward, servo position
```

## What maps to what

| Old | New |
| --- | --- |
| `Hardwares/RobotHardware` | `DriveSubsystem` + `LightSubsystem` |
| `Hardwares/ShooterHardware` | `ShooterSubsystem` + `IntakeSubsystem` |
| `Hardwares/LimelightHardware` | `VisionSubsystem` |
| `Teleops/OnePersonDrive*` | `teleop/` |
| `Teleops/PIDTuning` | `tuning/ShooterPidTuningTeleOp` |
| `Teleops/Flywheel_Tune` | `tuning/FlywheelTuningTeleOp` |
| `Teleops/ServoTest` | `tuning/ServoTuningTeleOp` |
| `Autos/*` + `RoadrunnerQuickstart/` | `autos/` on Pedro Pathing |

`Teleops/LimelightPrototype` has no opmode annotation and was already dead, so it
was not carried over — `VisionSubsystem` telemetry covers what it printed.

## Coordinates

Paths are written in **FTC standard coordinates** — the centre-origin, ±72 inch frame
the Road Runner autos used — and converted with `PedroConstants.ftcPose(x, y, heading)`.
The waypoints are therefore directly comparable to the old `Pose2d` literals.

Pedro's own frame has a different origin *and* a different heading zero, so never mix
the two: take headings off a converted `Pose` with `getHeading()` rather than writing
raw radians into a heading interpolator.

## Still needs the robot

The odometry geometry was converted from the Road Runner tuning, so it starts from
numbers measured on this robot. These could not be:

1. **Encoder directions** (`PedroConstants.localizerConfig`). Road Runner read the
   encoders raw; Pedro folds in the direction of the motor each encoder is plugged
   into, so the reversals do not carry over one for one. This was got wrong once
   already — the derivation is written out in the javadoc there. Push the robot
   forward a known distance and confirm the reported x matches, strafe left and
   confirm y grows.
2. **Max velocities and natural decelerations** (`PedroConstants`, the four
   `MAX_*` / `NATURAL_*` constants) are Pedro stock values. Run Pedro's velocity
   and zero-power-deceleration tuners.
3. **Foresight gains and brake coefficients** (`PedroConstants.foresightConfig`).
   The PID gains are Pedro 2's defaults carried across, so following starts where
   it did before. The brake coefficients are new in Pedro 3 with no Pedro 2
   analogue — they are seeded from coasting physics (`1/(2a)`, which predicts the
   `v^2/(2a)` stopping distance) and err toward slight overshoot. Run Pedro 3's
   autotuner.
4. **Auto paths.** Road Runner splines and Pedro Bezier curves do not reproduce
   each other exactly, and the old trajectories were hand-tuned, with several
   follow-up segments starting from approximated rather than computed poses. The
   waypoints and the sequencing carry over faithfully; the curves between them
   need a dry run.

The drivetrain watchdog in `DriveSubsystem` exists precisely because items 1–3 are
unverified: it cuts power if the follower stops making progress toward the path
endpoint, so a bad number costs a failed auto rather than a Control Hub.

## Behaviour changes worth knowing

- **Limelight is sampled once per loop** instead of once per getter. A shot solution
  can no longer be built from a distance and a tx that came from different frames.
- **The dpad velocity trim actually works.** The old teleop recomputed the target
  from the distance regression every loop, so its `+= 0.5` was overwritten before it
  could take effect. The trim is now kept separate from the solution.
- **Intake and extake are gated on the shoot button.** They drive the same roller the
  shot feeds with, and without the gate the two bindings would cancel and reschedule
  each other every loop.
- **`strafeCorrection = 1.45` is gone.** It compensated for the old hand-rolled
  mecanum kinematics; Pedro handles strafe scaling through `yVelocity`, which is what
  the lateral velocity tuner sets.
