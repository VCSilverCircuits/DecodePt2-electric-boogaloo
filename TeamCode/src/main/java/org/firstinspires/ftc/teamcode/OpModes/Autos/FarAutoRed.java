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
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Far Auto Red")
public class FarAutoRed extends OpMode {

    // Hardware
    private DcMotorEx leftFlywheel, rightFlywheel, intake;
    double timesShot = 0;


    private Follower follower;
    private MecanumConstants mecanumConstants;
    private OdoAim turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    AutoFlywheelConstants flywheel;
    private Timer pathTimer;

    // Pathing
    private Paths paths;
    private int pathState = 0;

    // Poses
    private static final Pose startPose = new Pose(84.037, 10.019, Math.toRadians(0));
    private static final Pose firingPose = new Pose(86, 26, Math.toRadians(0));
    private static final Pose intake1 = new Pose(128.897, 36, Math.toRadians(0));

    @Override
    public void init() {
        // Follower & drive constants
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        // Turret
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, true);
        // Sensors & servos
        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Timers
        pathTimer = new Timer();

        // Paths
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        flywheel.update(-follower.getVelocity().getXComponent() * 50);
        Pose robotPose = follower.getPose();

        servos.loop();
        turret.update();
        turret.odoAim();
        flywheel.enable();
        telemetry.addData("path state", pathState);
        telemetry.addData("current flywheel rpm", flywheel.getCurrentRPM());
        telemetry.addData("target flywheel rpm", flywheel.getTargetRPM());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("times Shot", timesShot);
        telemetry.update();
        // Pathing
        pathState = paths.autonomousPathUpdate(pathState, robotPose);
    }

    // Inner class for paths
    public class Paths {
        private PathChain startToFiring, firingToIntake1, intake1ToFiring;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;

            startToFiring = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firingPose.getHeading())
                .build();

            firingToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(firingPose,new Pose(86.05597663551401, 38.644859813084125), intake1))
                .setLinearHeadingInterpolation(firingPose.getHeading(), intake1.getHeading())
                .build();
            intake1ToFiring = follower.pathBuilder()
                .addPath(new BezierLine(intake1, firingPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), firingPose.getHeading())
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    follow.followPath(startToFiring);
                    pathState = 2;
                    break;


                case 2:
                    if (follower.atPose(firingPose, 2, 2) || pathTimer.getElapsedTimeSeconds() > 1.5) {
                         // prevents double start
                            if (!servos.isRunning() && pathTimer.getElapsedTimeSeconds() > 4) {
                                servos.StartNonSort();
                                timesShot++;
                                pathTimer.resetTimer();
                                pathState = 3;
                        }
                    }

                    break;


                case 3:
                    if (!servos.isRunning()) {
                        turret.idle();
                        if (timesShot < 2) {
                            intake.setPower(-1);
                            follow.followPath(firingToIntake1);
                            pathTimer.resetTimer();
                            pathState = 4;
                        } else { pathState = 7;
                        }
                    }
                    break;


                case 4:
                    if (follow.atPose(intake1, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                        pathTimer.resetTimer();  // reset here
                        follow.followPath(intake1ToFiring);
                        follower.setMaxPower(0.5);
                        pathState = 5;
                    }
                    break;

            case 5:
                turret.odoAim();
                pathTimer.resetTimer();
                pathState = 2;
            break;
               case 7:
                   PoseStorage.currentPose = follower.getPose();
                   turret.idle();
                   if (pathTimer.getElapsedTimeSeconds() > 4){
                       requestOpModeStop();
                   }
                    break;
            }
                return pathState;

            }
        }
    }





