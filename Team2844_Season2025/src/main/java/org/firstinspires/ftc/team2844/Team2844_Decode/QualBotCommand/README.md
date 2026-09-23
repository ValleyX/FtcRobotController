# QualBotCommand

The Qual Bot rewritten on [valleyLib](https://github.com/ValleyX/valleyLib) command-based,
with autonomous on Pedro Pathing instead of Road Runner.

This sits alongside the original `QualBot` package rather than replacing it, so the
old LinearOpModes still run while this is being validated. Every new opmode name is
prefixed `CB` so the two sets never collide on the Driver Station.

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

1. **Encoder directions** (`PedroConstants.localizerConstants`). Road Runner read the
   encoders raw; Pedro folds in the direction of the motor each encoder is plugged
   into, so the reversals do not carry over one for one. Push the robot forward and
   confirm x grows, strafe left and confirm y grows.
2. **`xVelocity` / `yVelocity`** (`PedroConstants.mecanumConstants`) are Pedro stock
   values. Run Pedro's Forward and Lateral Velocity tuners.
3. **`mass` and the zero-power accelerations** (`PedroConstants.followerConstants`)
   are also stock. Weigh the robot and run the zero-power-acceleration tuners.
4. **Auto paths.** Road Runner splines and Pedro Bezier chains do not reproduce each
   other exactly, and the old trajectories were hand-tuned, with several follow-up
   segments starting from approximated rather than computed poses. The waypoints and
   the sequencing carry over faithfully; the curves between them need a dry run.

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
