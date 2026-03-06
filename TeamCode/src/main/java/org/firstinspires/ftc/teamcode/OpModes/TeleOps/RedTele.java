package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.TeleFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red Tele")
public class RedTele extends OpMode {

    private TeleFlywheelConstants flywheel;
    private Follower follower;
    private OdoAim odoAim;
    private ServoGroup servos;
    private ColorSensors sensors;

    private DcMotorEx intake;

    private Servo lift1, lift2;
    private Servo stopper;
    private Servo servo1, servo2, servo3;

    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    private boolean liftToggle = false;
    private boolean lastLiftToggle = false;

    private boolean isFiring = false;
    private boolean intakeLocked = false;

    // ================= TURRET CONTROL =================
    private boolean turretTrackingEnabled = false;
    private boolean lastTurretButton = false;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(PoseStorage.currentPose);

        odoAim = new OdoAim(hardwareMap, follower, true);

        // Initially idle, hold current position
        odoAim.idle();

        flywheel = new TeleFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");

        servo1.setPosition(0.05);
        servo2.setPosition(0.05);
        servo3.setPosition(0.05);

        servos = new ServoGroup(
            hardwareMap,
            "frontFlipper",
            "backFlipper",
            "leftFlipper",
            "stopper"
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        stopper = hardwareMap.get(Servo.class, "stopper");

        lift1.setPosition(0.9);
        lift2.setPosition(0.92);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(PoseStorage.currentPose);
        flywheel.enable();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );

        follower.update();
        flywheel.update(-gamepad1.left_stick_y * 50);

        odoAim.update();
        servos.loop();

        // -------- TOGGLE TRACKING --------
        boolean turretButtonPressed = gamepad1.left_bumper;

        if (turretButtonPressed && !lastTurretButton) {
            turretTrackingEnabled = !turretTrackingEnabled;

            if (turretTrackingEnabled) {
                // Reset tracking zero to current idle position
                odoAim.syncToCurrentPosition();
            } else {
                // When turning OFF, hold current position
                odoAim.idle();
            }
        }
        lastTurretButton = turretButtonPressed;

        // -------- OFFSET CONTROLS --------
        if (gamepad1.dpad_left && !lastDpadLeft) {
            odoAim.changeTarget(true, false);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            odoAim.changeTarget(false, true);
        }

        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;

        // -------- AIM ONLY IF ENABLED --------
        if (turretTrackingEnabled) {
            odoAim.odoAim();
        } else {
            odoAim.idle();
        }
        if (gamepad1.dpad_down){
            odoAim.recalibration(follower);
            flywheel.recalibration(follower);
        }


        // ================= INTAKE =================
        boolean intakePressed = gamepad1.left_trigger > 0.5;
        boolean backspinPressed = gamepad1.right_trigger > 0.5;

        if (!intakeLocked) {
            if (intakePressed && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
            }
            lastIntakeTrigger = intakePressed;

            if (backspinPressed) {
                intake.setPower(1);
            } else if (intakeToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        } else {
            intake.setPower(0);
        }

        // ================= FIRING =================
        if (gamepad1.y && !isFiring) {
            isFiring = true;
            intakeLocked = true;
            intake.setPower(0);
            sensors.reset();
            sensors.update();
            servos.StartNonSort();
        }

        if (!servos.isRunning() && isFiring) {
            isFiring = false;
            intakeLocked = false;
        }

        if (servos.isRunning()) {
            stopper.setPosition(0.3);
        } else {
            stopper.setPosition(0);
        }

        if (!isFiring) {
            servo1.setPosition(gamepad1.b ? 1 : 0);
            servo2.setPosition(gamepad1.a ? 1 : 0);
            servo3.setPosition(gamepad1.x ? 1 : 0);
        }

        // ================= LIFT TOGGLE =================
        boolean liftPressed = gamepad2.dpad_up;

        if (liftPressed && !lastLiftToggle) {
            liftToggle = !liftToggle;
        }
        lastLiftToggle = liftPressed;

        if (liftToggle) {
            lift1.setPosition(0.5);
            lift2.setPosition(0.52);
        } else {
            lift1.setPosition(0.9);
            lift2.setPosition(0.92);
        }

        // ================= TELEMETRY =================
        telemetry.addData("Turret Tracking Enabled", turretTrackingEnabled);
        telemetry.addData("Turret Offset (deg)", odoAim.getOffsetDegrees());
        telemetry.update();
    }
}
