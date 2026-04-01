package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAimBlue;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Far Blue")
public class FarAutoBlue extends OpMode {

    // Hardware
    private DcMotorEx intake;
    double timesShot = 0;

    private Follower follower;
    private MecanumConstants mecanumConstants;
    private OdoAimBlue turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    private AutoFlywheelConstants flywheel;
    private Timer pathTimer;
    private Timer poseTimer;

    boolean intakeDelayStarted = false;
    boolean endTriggered = false;
    boolean isAtShootingPose = false;

    // Pathing
    private Paths paths;
    private int pathState = 0;

    // Poses
    private static final Pose startPose = new Pose(59.963, 10.019, Math.toRadians(180));
    private static final Pose firingPose = new Pose(53, 10.019, Math.toRadians(180));

    private static final Pose intake1 = new Pose(15.103, 36, Math.toRadians(180));
    private static final Pose intake2 = new Pose(10, 12.019, Math.toRadians(180));
    private static final Pose endPose = new Pose(10, 30, Math.toRadians(180));

    @Override
    public void init() {

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        turret = new OdoAimBlue(hardwareMap, follower, false);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, false);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        poseTimer = new Timer();

        paths = new Paths(follower);
    }

    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.getPose();

        // ✅ Check if at shooting pose
        isAtShootingPose = follower.atPose(firingPose, 7, 7);



        flywheel.update(-follower.getVelocity().getXComponent() * 50);
        flywheel.setConstantRPM(4250);
        // ===== TURRET CONTROL =====
        if (isAtShootingPose) {
            turret.odoAim();
        } else {
            turret.idle();
        }

        servos.loop();
        turret.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("times Shot", timesShot);
        telemetry.addData("pose", robotPose);
        telemetry.addData("at shooting Pose", isAtShootingPose);
        telemetry.addData("is running?", servos.isRunning());
        telemetry.addData("current flywheel rpm", flywheel.getCurrentRPM());
        telemetry.addData("target flywheel rpm", flywheel.getTargetRPM());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

        // End condition
        if (!endTriggered && poseTimer.getElapsedTimeSeconds() >= 28.5) {
            endTriggered = true;
            follower.followPath(paths.firingToEnd);
        }
        if (endTriggered && pathTimer.getElapsedTimeSeconds() >=30){
            PoseStorage.currentPose = follower.getPose();
            PoseStorage.turretRadians = turret.getTurretPosition();
        }

        if (!endTriggered) {
            pathState = paths.autonomousPathUpdate(pathState, robotPose);
        }
    }

    public class Paths {

        private PathChain startToFiring;

        private PathChain firingToIntake1;
        private PathChain intake1ToFiring;

        private PathChain firingToIntake2;
        private PathChain intake2ToFiring;
        private PathChain firingToEnd;

        private Follower follow;

        public Paths(Follower follower) {

            this.follow = follower;

            startToFiring = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firingPose.getHeading())
                .build();

            firingToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(firingPose,
                    new Pose(57.95, 38.64),
                    intake1))
                .setLinearHeadingInterpolation(firingPose.getHeading(), intake1.getHeading())
                .build();

            intake1ToFiring = follower.pathBuilder()
                .addPath(new BezierLine(intake1, firingPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), firingPose.getHeading())
                .build();

            firingToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(firingPose, intake2))
                .setLinearHeadingInterpolation(firingPose.getHeading(), intake2.getHeading())
                .build();

            intake2ToFiring = follower.pathBuilder()
                .addPath(new BezierLine(intake2, firingPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), firingPose.getHeading())
                .build();

            firingToEnd = follower.pathBuilder()
                .addPath(new BezierLine(firingPose, endPose))
                .setLinearHeadingInterpolation(firingPose.getHeading(), endPose.getHeading())
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {

            switch (pathState) {

                case 0:
                    pathTimer.resetTimer();
                    poseTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    follow.followPath(startToFiring);
                    pathState = 2;
                    break;

                case 2:
                    intake.setPower(1);

                    if (!servos.isRunning()
                        && follow.atPose(firingPose, 2, 2)
                        && flywheel.atSpeed(200)) {

                        servos.StartNonSort();
                        timesShot++;

                        pathTimer.resetTimer();
                        pathState = 3;
                    } else if (pathTimer.getElapsedTimeSeconds() > 3 && flywheel.atSpeed(200) ){
                        servos.StartNonSort();
                        timesShot++;

                        pathTimer.resetTimer();
                        pathState = 3;
                    }
                    break;

                case 3:

                    if (!servos.isRunning()) {

                        if (timesShot == 1) {

                            intake.setPower(-1);
                            follow.followPath(firingToIntake1);

                            pathTimer.resetTimer();
                            pathState = 4;
                        } else if (timesShot >= 2) {

                            intake.setPower(-1);

                            follower.setMaxPower(1);
                            follow.followPath(firingToIntake2);

                            pathTimer.resetTimer();
                            pathState = 6;
                        } else {
                            pathState = 7;
                        }
                    }
                    break;

                case 4:
                    intake.setPower(-1);

                    if (follow.atPose(intake1, 2, 2) || pathTimer.getElapsedTimeSeconds() > 3) {

                        if (!intakeDelayStarted) {
                            pathTimer.resetTimer();
                            intakeDelayStarted = true;
                        }

                        follow.followPath(intake1ToFiring);

                        if (pathTimer.getElapsedTimeSeconds() > 1) {

                            intake.setPower(1);

                            intakeDelayStarted = false;

                            pathTimer.resetTimer();

                            follower.setMaxPower(0.8);
                            pathState = 5;
                        }
                    }
                    break;

                case 5:
                    pathState = 2;
                    break;

                case 6:

                    intake.setPower(-1);

                    if (follow.atPose(intake2, 2, 2) || pathTimer.getElapsedTimeSeconds() > 3) {

                        if (!intakeDelayStarted) {
                            pathTimer.resetTimer();
                            intakeDelayStarted = true;
                        }

                        pathState = 8;
                    }
                    break;

                case 8:

                    intake.setPower(-1);
                    follow.followPath(intake2ToFiring);
                    follower.setMaxPower(0.8);

                    if (pathTimer.getElapsedTimeSeconds() > 1) {

                        intake.setPower(1);

                        intakeDelayStarted = false;
                        pathTimer.resetTimer();
                        pathState = 5;
                    }
                    break;
            }

            return pathState;
        }
    }
}
