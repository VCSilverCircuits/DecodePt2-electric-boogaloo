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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.Subsystems.DualMotor;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Red Auto Shoot")
public class CloseRedAuto extends OpMode {
    PoseStorage poseStorage;
    private Follower follower;
    private Timer pathTimer;
    private Timer leaveTimer;
    MecanumConstants mecanumConstants;

    private DcMotorEx turret;
    public static final double TICKS_PER_REV = 28;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 53;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    int timesShot = 0;

    private AprilTagTurretControllerRed turretController;

    double endPoseY = 93.9813;
    double endPoseX = 95.1028;
    private final Pose startPose = new Pose(122.0187, 123.8131, Math.toRadians(37));
    private final Pose endPose = new Pose(endPoseX, endPoseY, Math.toRadians(37));
    private final Pose turnToIntake = new Pose(90.39252336448597, 89.27101962616824, Math.toRadians(0));
    private final Pose intake1 = new Pose(124, 96, Math.toRadians(0));
    private final Pose releaseBalls = new Pose(130, 80, Math.toRadians(0));
    private final Pose intake2Lineup = new Pose(95.15887850467287,64,Math.toRadians(-3));
    private final Pose intake2 = new Pose(130,62, Math.toRadians(-7));
    private final Pose intake3Lineup = new Pose(94.75700934579439,42,Math.toRadians(0));
    private final Pose intake3 = new Pose(132,42, Math.toRadians(-3));
    private DcMotorEx leftFlywheel, rightFlywheel;
    private DualMotor flywheel;
    private Servo servo1, servo2, servo3, servo4;
    private DcMotorEx intake;


    boolean comingBack = false;

    public static final double MAX_FLYWHEEL_RPM = 6000;

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        // Flywheel
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mecanumConstants = new MecanumConstants(); // assign to the class-level field


        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new AprilTagTurretControllerRed(hardwareMap);
        turretController.resetController();

        // Follower
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);
        follower.update();
        poseStorage = new PoseStorage(follower);

        // Turret
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Servos
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");
        servo4 = hardwareMap.get(Servo.class, "stopper");
        servo1.setPosition(0.05);
        servo2.setPosition(0.05);
        servo3.setPosition(0.05);
        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Timer
        pathTimer = new Timer();
        leaveTimer = new Timer();

        // Paths
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        follower.update();
        Pose robotPose = follower.getPose();
        poseStorage.Update();
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        double power = turretController.getTurretPower(currentAngleDeg);

        // ONLY run AprilTag tracking during shooting states
        if (pathState == 2 || pathState == 3 || pathState == 4) {
            turret.setPower(power);
        } else {
            turret.setPower(0);
        }

        pathState = paths.autonomousPathUpdate(pathState, robotPose);

        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Turret Angle", currentAngleDeg);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Path State", pathState);
        telemetry.addData("shootsFired!!", timesShot);
        telemetry.addData("leave Timer", leaveTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void start(){
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        turretController.setPosition(0, currentAngleDeg);
    }

    public class Paths {

        private PathChain startToEnd, endToTurnToIntake, turnToIntakeToIntake1, intake1ToReleaseBalls, releaseBallsToEndPose, endPoseToLineUp, lineupToIntake2, intake2ToEndPose, endPoseToLineup3, lineup3ToIntake3, intake3ToEndPose;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;

            startToEnd = follower.pathBuilder().addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading()).build();

            endToTurnToIntake = follower.pathBuilder().addPath(new BezierLine(endPose, turnToIntake))
                .setLinearHeadingInterpolation(endPose.getHeading(), intake1.getHeading()).build();

            turnToIntakeToIntake1 = follower.pathBuilder().addPath(new BezierLine(turnToIntake, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            intake1ToReleaseBalls = follower.pathBuilder().addPath(new BezierCurve(intake1, new Pose(115.107476635514, 80.23831775700934), releaseBalls))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            releaseBallsToEndPose = follower.pathBuilder().addPath(new BezierLine(releaseBalls, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37)).build();
            endPoseToLineUp = follower.pathBuilder().addPath(new BezierLine(endPose,intake2Lineup))
                .setLinearHeadingInterpolation(Math.toRadians(37),Math.toRadians(-3)).build();

            lineupToIntake2 = follower.pathBuilder().addPath(new BezierLine(intake2Lineup, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-3)).build();

            intake2ToEndPose = follower.pathBuilder().addPath(new BezierCurve(intake2, new Pose(98.01869158878506,57.25233644859814),endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37)).build();
            endPoseToLineup3 = follower.pathBuilder().addPath(new BezierLine(endPose, intake3Lineup))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
            lineup3ToIntake3 = follower.pathBuilder().addPath(new BezierLine(intake3Lineup, intake3))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0)).build();
            intake3ToEndPose = follower.pathBuilder().addPath(new BezierLine(intake3, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37)).build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0: // Start -> Shoot preload
                    leaveTimer.resetTimer();
                    follow.followPath(startToEnd);
                    follower.setMaxPower(1);
                    setFlywheelRPM(3650);
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    if (follow.atPose(endPose, 2, 2)) {
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
                    break;

                case 2: // Shoot
                    intake.setPower(1);
                    servo4.setPosition(1);
                    if (leaveTimer.getElapsedTimeSeconds() >= 29) {
                        endPoseX = endPoseX - 5;
                    }
                    intake.setVelocity(0);
                    if (pathTimer.getElapsedTimeSeconds() > 0.4 && pathTimer.getElapsedTimeSeconds() < 0.8) {
                        servo1.setPosition(1);
                        //0
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                        servo1.setPosition(0.05);
                        //0.45
                        pathTimer.resetTimer();
                        pathState = 3;
                    }
                    break;

                case 3:
                    if (leaveTimer.getElapsedTimeSeconds() >= 29) {
                        endPoseX = endPoseX - 5;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.4 && pathTimer.getElapsedTimeSeconds() < 0.8) {
                        servo2.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                        servo2.setPosition(0.05);
                        pathTimer.resetTimer();
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (leaveTimer.getElapsedTimeSeconds() >= 29) {
                        endPoseX = endPoseX - 5;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.4 && pathTimer.getElapsedTimeSeconds() < 0.8) {
                        servo3.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 0.8) {
                        servo3.setPosition(0.05);
                        pathTimer.resetTimer();
                        intake.setPower(-1);
                        servo4.setPosition(0.05);

                        // increment timesShot once
                        timesShot++;

                        if (timesShot == 1) {
                            pathState = 5; // go to first intake
                        } else if (timesShot == 2) {
                            pathState = 10; // go to second intake
                        } else if (timesShot == 3) {
                            pathState = 16; // final state after second round
                        } else {
                            pathState = 22;
                        }
                    }
                    break;


                // INTAKE FIRST CYCLE
                case 5:
                    follow.followPath(endToTurnToIntake);
                    pathState = 6;
                    break;

                case 6:
                    if (follow.atPose(turnToIntake, 2, 2)) {
                        follower.setMaxPower(0.8);
                        follow.followPath(turnToIntakeToIntake1);
                        pathTimer.resetTimer();
                        pathState = 7;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.setMaxPower(0.8);
                        follow.followPath(turnToIntakeToIntake1);
                        pathTimer.resetTimer();
                        pathState = 7;
                    }
                    break;

                case 7:
                    if (follow.atPose(intake1, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            intake.setPower(1);
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                            follow.followPath(intake1ToReleaseBalls);
                            follower.setMaxPower(0.7);
                            pathTimer.resetTimer();
                            pathState = 8;
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            intake.setPower(1);
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            follow.followPath(intake1ToReleaseBalls);
                            follower.setMaxPower(0.7);
                            pathTimer.resetTimer();
                            pathState = 8;
                        }
                    }
                    break;

                case 8:
                    if (follow.atPose(releaseBalls, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 2) {
                            intake.setPower(1);
                            follower.setMaxPower(1);
                            follow.followPath(releaseBallsToEndPose);
                            pathTimer.resetTimer();
                            pathState = 9;
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 2) {
                            intake.setPower(-1);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        intake.setPower(1);
                        follower.setMaxPower(1);
                        follow.followPath(releaseBallsToEndPose);
                        pathTimer.resetTimer();
                        pathState = 9;
                    }
                    break;

                case 9:
                    if (follow.atPose(endPose, 2, 2)) {
                        intake.setPower(0);
                        pathTimer.resetTimer();
                        pathState = 2; // final idle
                    }
                    break;
                case 10:
                    follow.followPath(endPoseToLineUp);
                    pathTimer.resetTimer();
                    pathState = 11;
                    break;
                case 11:
                    comingBack = true;
                    if (follow.atPose(intake2Lineup, 2, 2)) {
                        if (comingBack = true) {
                            endPoseX = endPoseX - 3;
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 1.75) {
                            pathState = 12;
                            pathTimer.resetTimer();
                        }

                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        pathTimer.resetTimer();
                        pathState = 12;
                    }
                    break;


                case 12: // Move to second intake
                    follow.followPath(lineupToIntake2);
                    follower.setMaxPower(0.75);
                    intake.setPower(-1);
                    pathTimer.resetTimer(); // reset timer here once
                    pathState = 13;
                    break;

                case 13: // Intake 2
                    follower.setMaxPower(1);
                    if (follow.atPose(intake2, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 2) {
                            intake.setPower(1);
                            pathTimer.resetTimer();
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                            pathTimer.resetTimer();
                            pathState = 14;
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        pathState = 14;
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            intake.setPower(1);
                            pathTimer.resetTimer();
                        }
                    }
                    follower.setMaxPower(1);
                    endPoseX = endPoseX + 2;
                    endPoseY = endPoseY + 2;
                    break;
                case 14:
                    follow.followPath(intake2ToEndPose);
                    pathState = 15;
                   break;
                case 15:
                    if (follow.atPose(endPose,2,2) ){
                        endPoseX = endPoseX+3;
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
                    break;
                case 16:
                    follow.followPath(endPoseToLineup3);
                    pathTimer.resetTimer();
                    pathState = 17;
                    break;
                case 17:
                    if (follow.atPose(intake3Lineup,2,2)){
                        follower.setMaxPower(0.75);
                        intake.setPower(-1);
                        pathTimer.resetTimer();
                        pathState = 18;
                    }
                        break;
                case 18:
                    follower.followPath(lineup3ToIntake3);
                    follower.setMaxPower(0.75);
                    pathTimer.resetTimer();
                    pathState = 19;
                    break;
                case 19:
                    follower.setMaxPower(1);
                    if (follow.atPose(intake3,2,2)){
                        if (pathTimer.getElapsedTimeSeconds() > 2){
                            intake.setPower(1);
                            pathTimer.resetTimer();
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                            pathTimer.resetTimer();
                            pathState = 20;
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5){
                        pathState = 20;
                        if (pathTimer.getElapsedTimeSeconds() > 3){
                            intake.setPower(1);
                            pathTimer.resetTimer();
                        }
                    }
                    break;
                case 20:
                    follow.followPath(intake3ToEndPose);
                    follower.setMaxPower(1);
                    pathTimer.resetTimer();
                    pathState = 21;
                    break;
                case 21:
                    if (follow.atPose(endPose,2,2)){
                        pathTimer.resetTimer();
                        intake.setPower(0);
                        pathState = 2;
                    }
                    break;
                case 22:
                    follower.setMaxPower(1);
                    setFlywheelRPM(3800);
                    intake.setPower(0);
                    if (leaveTimer.getElapsedTimeSeconds() >= 28){
                        follower.followPath(endPoseToLineup3);
                    }
                    break;
            }
            return pathState;
        }

        private void setFlywheelRPM(double rpm) {
            double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
            leftFlywheel.setVelocityPIDFCoefficients(0.022,0,0,13.6);
            leftFlywheel.setVelocity(ticksPerSecond);
            rightFlywheel.setVelocity(ticksPerSecond);
        }
    }
}
