# VC Silver Circuits 16158 - 2025-2026  DECODE FTC Robot Code

> # NOTE
> **This repository is public for transparency only.** This is our team's competition robot code. It is not intended as a general-purpose template or starter project for other teams. Feel free to learn from it, but it is written specifically for our robot's hardware configuration and may not work for yours.

FTC robot controller code for **VC Silver Circuits (Team 16158)** for the 2025-2026 season. Built on the standard FTC SDK with [PedroPathing](https://pedropathing.com/) for autonomous path following.

**Website:** [https://vcsilvercircuits.com](https://www.vcsilvercircuits.com/)
**Robot Info:** [https://vcsilvercircuits.com/decode-robot](https://vcsilvercircuits.com/decode-robot)
**Questions:** [programming@vcsilvercircuits.com](mailto:programming@vcsilvercircuits.com)

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Hardware Map](#hardware-map)
- [OpModes](#opmodes)
- [Subsystems](#subsystems)
- [Teleop Controls](#teleop-controls)
- [Autonomous Sequences](#autonomous-sequences)
- [Tuning Parameters](#tuning-parameters)
- [Hot Reload with Sloth](#hot-reload-with-sloth)

---

## Overview

The robot is a **mecanum drive** platform equipped with:

- A **flywheel shooter** that adjusts RPM and hood angle automatically based on distance to the target
- A **rotating turret** that aims at the goal using odometry or Limelight 3A visual tracking
- Three **flipper servos** that push game pieces (green/purple) into the shooter
- An **intake** roller motor
- **Color sensors** on each flipper to identify which game piece is in each slot
- A **Limelight 3A** camera for AprilTag-based turret targeting and motif pattern detection
- A **GoBilda Pinpoint** odometry pod for field-relative localization

---

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── OpModes/
│   ├── Autos/
│   │   ├── CloseBlueAuto.java       # Blue alliance close-side auto
│   │   ├── CloseRedAuto.java        # Red alliance close-side auto
│   │   ├── FarAutoBlue.java         # Blue alliance far-side auto
│   │   └── FarAutoRed.java          # Red alliance far-side auto
│   ├── TeleOps/
│   │   ├── BlueTele.java            # Main Blue alliance teleop
│   │   ├── RedTele.java             # Main Red alliance teleop
│   │   ├── OldBlueTele.java         # Legacy Blue teleop
│   │   ├── OldRedTele.java          # Legacy Red teleop
│   │   └── ScrimBotTelop.java       # Scrimmage teleop
│   ├── TestingAutos/
│   │   └── MotifScannerAuto.java    # Auto that scans and stores the motif pattern
│   └── TestingTeleOps/
│       ├── MotifShooterTeleOp.java  # Teleop for testing motif-based sorting
│       ├── UrsaTestTele.java        # General hardware testing teleop
│       └── MegaTagTesting.java      # Limelight MegaTag testing
├── Subsystems/
│   ├── AprilTagControllers/
│   │   ├── AprilTagTurretControllerBlue.java
│   │   └── AprilTagTurretControllerRed.java
│   ├── ColorSensorTests/
│   │   └── ColorSensors.java        # Reads 3 color sensors, classifies green/purple
│   ├── FlywheelConstants/
│   │   ├── AutoFlywheelConstants.java
│   │   ├── BlueTeleFlywheelConstants.java
│   │   └── TeleFlywheelConstants.java
│   ├── Motif/
│   │   ├── MatchMotif.java          # Shared static state for the detected motif pattern
│   │   ├── MotifDetector.java       # Reads Limelight AprilTags to detect motif
│   │   └── ServoGroup.java          # Sequences flipper servos (sorted or non-sorted)
│   ├── DcMotorGroup.java
│   ├── DualMotor.java
│   ├── OdoAim.java                  # Red alliance turret aiming (odo + Limelight)
│   ├── OdoAimBlue.java              # Blue alliance turret aiming (odo + Limelight)
│   └── PoseStorage.java             # Persists robot pose and turret angle from auto to teleop
└── pedroPathing/
    ├── AutoConstants.java           # PedroPathing config for autonomous
    ├── Constants.java               # PedroPathing config for teleop
    └── Tuning.java                  # Path tuning OpMode
```

---

## Hardware Map

All hardware names must match exactly in the Driver Hub configuration.

### Motors

| Config Name      | Hub           | Port | Purpose                          | Notes                              |
|------------------|---------------|------|----------------------------------|------------------------------------|
| `frontLeft`      | Control Hub   | 3    | Drive - front left               | FORWARD direction                  |
| `frontRight`     | Expansion Hub | 0    | Drive - front right              | REVERSE direction                  |
| `backLeft`       | Control Hub   | 2    | Drive - rear left                | FORWARD direction                  |
| `backRight`      | Expansion Hub | 1    | Drive - rear right               | REVERSE direction                  |
| `turretRotation` | Control Hub   | 0    | Turret yaw motor                 | REVERSE direction, no encoder mode |
| `Output1`        | Expansion Hub | 2    | Flywheel - left                  | REVERSE direction                  |
| `Output2`        | Expansion Hub | 3    | Flywheel - right                 |                                    |
| `intake`         | Control Hub   | 1    | Intake roller                    | BRAKE on zero power                |

### Servos

| Config Name    | Hub       | Port | Purpose                                        | Default Position |
|----------------|-----------|------|------------------------------------------------|-----------------|
| `frontFlipper` | Servo Hub | 1    | Pushes game piece from front slot into shooter | 0.05 (down)     |
| `backFlipper`  | Servo Hub | 5    | Pushes game piece from back slot into shooter  | 0.05 (down)     |
| `leftFlipper`  | Servo Hub | 2    | Pushes game piece from left slot into shooter  | 0.05 (down)     |
| `stopper`      | Servo Hub | 4    | Gates game pieces before the flywheels         | 0 (open)        |
| `Hood`         | Expansion Hub | 0 | Adjusts flywheel angle                        | Auto-calculated  |
| `lift1`        | Servo Hub | 3    | Endgame lift - servo 1                         | 0.9 (down)      |
| `lift2`        | Servo Hub | 0    | Endgame lift - servo 2                         | 0.92 (down)     |

### Sensors

| Config Name  | Hub           | Port / Bus   | Purpose                                 |
|--------------|---------------|--------------|-----------------------------------------|
| `pinpoint`   | Control Hub   | I2C bus 2    | GoBilda Pinpoint odometry pod           |
| `limelight`  | Ethernet      | 172.29.0.1   | Limelight 3A (turret aiming + motif)    |
| `sensorF1`   | Expansion Hub | I2C bus 0    | Color sensor - front flipper slot       |
| `sensorF2`   | Expansion Hub | I2C bus 1    | Color sensor - front flipper slot (2)   |
| `sensorB1`   | Expansion Hub | I2C bus 3    | Color sensor - back flipper slot        |
| `sensorB2`   | Expansion Hub | I2C bus 2    | Color sensor - back flipper slot (2)    |
| `sensorLL`   | Control Hub   | I2C bus 3    | Color sensor - left flipper slot        |
| `sensorLR`   | Control Hub   | I2C bus 1    | Color sensor - right flipper slot       |
| `imu`        | Control Hub   | I2C bus 0    | Built-in IMU (BHI260AP)                 |

---

## OpModes

### Teleop

| Name             | Alliance | Description                                      |
|------------------|----------|--------------------------------------------------|
| `Blue Tele`      | Blue     | Main match teleop, imports pose from auto        |
| `Red Tele`       | Red      | Main match teleop, imports pose from auto        |
| `ScrimBotTelop`  | Either   | Simplified teleop used for scrimmages            |

### Autonomous

| Name               | Alliance | Start Position  | Description                                        |
|--------------------|----------|-----------------|----------------------------------------------------|
| `Blue Auto Shoot`  | Blue     | Close side       | Shoots preload + 3 intake cycles                  |
| `Red Auto Shoot`   | Red      | Close side       | Mirrored version of Blue close auto               |
| `Far Auto Blue`    | Blue     | Far side         | Far-side starting position auto                   |
| `Far Auto Red`     | Red      | Far side         | Far-side starting position auto                   |

### Testing / Tuning

| Name                  | Description                                              |
|-----------------------|----------------------------------------------------------|
| `MotifScannerAuto`    | Scans backdrop AprilTags and stores the motif pattern   |
| `MotifShooterTeleOp`  | Tests motif-sorted flipper firing sequence              |
| `UrsaTestTele`        | General hardware/subsystem testing                      |
| `MegaTagTesting`      | Tests Limelight MegaTag localization                    |

---

## Subsystems

### OdoAim / OdoAimBlue

Turret aiming controller that keeps the flywheel pointed at the goal.

- Reads robot pose from PedroPathing's odometry follower
- Computes the field-relative angle from the robot to the target goal pose
- Converts to a robot-relative heading and drives the turret via a PIDF controller
- Falls back to **Limelight 3A** visual tracking when a valid target is in frame
- Supports a **manual offset** (D-pad left/right) to nudge the aim without disabling tracking
- Hard limits: -90 deg to +175 deg turret rotation

**Target poses (field coordinates, inches):**
- Red goal: (152, 146)
- Blue goal: (-3, 140)

### BlueTeleFlywheelConstants / TeleFlywheelConstants

Distance-based flywheel controller. Each update cycle it:

1. Computes the Euclidean distance from the robot to the target goal
2. Uses a **linear regression** to calculate the optimal RPM and hood angle for that distance
3. Applies a forward-velocity compensation term to account for robot movement while shooting
4. Drives both flywheel motors and the hood servo

**Regression constants (Blue):**
- RPM = 21.35 * distance + 2634.3
- Hood angle (deg) = -0.851 * distance + 139.1

### ServoGroup

Manages the sequential firing of the three flipper servos.

- **`StartNonSort()`** — fires servos in fixed order: front → back → left. Used in most match scenarios.
- **`startMotif()`** — fires servos in the order dictated by the detected motif pattern, matching the right colored piece to the right slot. Requires `MatchMotif` to be set and `ColorSensors` to have latched colors.
- Each servo is raised for 275 ms then lowered with a 275 ms delay between servos.
- The `stopper` servo gates the output channel during the sequence.

### MatchMotif

Static shared state that stores the motif pattern detected during autonomous (`GPP`, `PGP`, or `PPG` — combinations of Green and Purple game pieces). Teleop reads this to fire the correct flipper first.

### MotifDetector

Uses the Limelight 3A to detect AprilTags during autonomous:

| AprilTag ID | Pattern |
|-------------|---------|
| 21          | GPP (Green, Purple, Purple) |
| 22          | PGP (Purple, Green, Purple) |
| 23          | PPG (Purple, Purple, Green) |

### ColorSensors

Three `NormalizedColorSensor` instances that classify each flipper slot as **GREEN**, **PURPLE**, or **UNKNOWN** based on normalized R/G/B ratios. Values are latched on first detection and only reset manually.

### PoseStorage

Simple static class that persists two values across OpMode transitions (auto → teleop):
- `currentPose` — robot field pose at end of auto
- `turretRadians` — turret angular position at end of auto

---

## Teleop Controls

### Gamepad 1 (Driver)

| Input             | Action                                              |
|-------------------|-----------------------------------------------------|
| Left stick        | Drive (field-relative)                              |
| Right stick X     | Rotate (50% speed)                                  |
| Left bumper       | Toggle turret auto-tracking on/off                  |
| D-pad left        | Nudge turret aim target -3 inches in X              |
| D-pad right       | Nudge turret aim target +3 inches in X              |
| D-pad down        | Recalibrate turret and flywheel target positions    |
| Left trigger      | Toggle intake on/off                                |
| Right trigger     | Run intake reverse (backspin)                       |
| Y button          | Fire all three flippers (non-sorted, sequential)    |
| B button          | Manually flip front flipper up                      |
| A button          | Manually flip back flipper up                       |
| X button          | Manually flip left flipper up                       |

### Gamepad 2 (Operator)

| Input       | Action                    |
|-------------|---------------------------|
| D-pad up    | Toggle lift up/down       |

---

## Autonomous Sequences

### Close Blue Auto (`CloseBlueAuto`)

**Starting pose:** Right side of the field (flipped from Red coordinates), heading ~143 degrees.

**Sequence:**
1. **State 0** — Drive to shooting position, spin flywheels to 3650 RPM, run intake in
2. **States 1-4** — Wait to reach shooting pose, fire flippers sequentially (front → back → left) using AprilTag turret tracking
3. **States 5-9** — Drive to first intake position, intake game pieces, return and shoot again
4. **States 10-15** — Drive to second intake position, intake, return and shoot
5. **States 16-21** — Drive to third intake position, intake, return and shoot
6. **State 22** — Hold at shooting pose until end of match, save final pose to `PoseStorage`

The turret runs AprilTag tracking only during shooting states (2, 3, 4).

### Red Auto

Mirrors the Blue auto using coordinate flipping (`flipX` / `flipHeading` helpers that reflect poses across the center line of the 144" × 144" field).

---

## Tuning Parameters

### PedroPathing (Constants.java / AutoConstants.java)

| Parameter                       | Value     | Description                             |
|---------------------------------|-----------|-----------------------------------------|
| `forwardZeroPowerAcceleration`  | -42.19    | Deceleration tuning for forward motion  |
| `lateralZeroPowerAcceleration`  | -209.03   | Deceleration tuning for lateral motion  |
| `xVelocity`                     | 74.09     | Max forward velocity (in/s)             |
| `yVelocity`                     | 56.58     | Max lateral velocity (in/s)             |
| `forwardPodY`                   | 2.5 in    | Pinpoint forward pod offset             |
| `strafePodX`                    | 7.0 in    | Pinpoint strafe pod offset              |

### Turret PIDF (OdoAim.java)

| Controller        | P    | I   | D     | F   |
|-------------------|------|-----|-------|-----|
| Odometry PIDF     | 2.4  | 0.0 | 0.008 | 0.2 |
| Limelight PIDF    | 0.06 | 0.0 | 0.008 | 0.0 |

`RADIANSPERTICK = 0.001062` — converts motor encoder ticks to turret radians.

### Flywheel PIDF (BlueTeleFlywheelConstants.java)

| P      | I     | D       | F      |
|--------|-------|---------|--------|
| 0.0085 | 0.007 | 0.00002 | 0.0004 |

### Autonomous Flywheel RPM

| Auto             | RPM  |
|------------------|------|
| Close autos      | 3650 |
| Final hold state | 3800 |

---

## Setup

1. Clone the repo into Android Studio.
2. Connect to the Robot Controller via USB or Wi-Fi.
3. Verify all hardware config names match the [Hardware Map](#hardware-map) table above.
4. Run `Tuning` OpMode to verify PedroPathing constants for the current robot.
5. Run `MotifScannerAuto` in pre-match to detect and store the motif pattern before running teleop.
6. For competition matches, run the appropriate auto first, then select the matching alliance teleop — pose is automatically transferred via `PoseStorage`.

---

## Hot Reload with Sloth

This project uses [Sloth](https://github.com/Dairy-Foundation/Sloth) for hot code reloading. Instead of a full 40+ second APK install, Sloth pushes only changed TeamCode classes to the robot in under 2 seconds.

### Daily workflow

1. Make code changes in `TeamCode`
2. Run the **`deploySloth`** run configuration in Android Studio
3. Restart the OpMode on the Driver Hub — changes are live immediately

### Full install (required when)

- Adding or updating library dependencies
- Modifying files outside of `TeamCode` (e.g. `build.gradle`, `FtcRobotController`)
- First-time setup on a new robot

For a full install, run `removeSlothRemote` first (added as a "Before launch" step in the TeamCode run config), then do a normal build & deploy.

### Notes

- Only classes under `org.firstinspires.ftc.teamcode` are hot-reloaded
- Classes annotated with `@Pinned` are excluded from hot reload and always require a full install
- Sloth changes survive robot restarts and power cycles
